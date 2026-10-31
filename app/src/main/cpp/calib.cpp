// 见 calib.h。
//
// 只估 fx fy cx cy k1：k2/k3/切向在 640×480 上辨识不出来，钉住反而让 fx 更稳。
// 物体点是单位间距的平面网格 —— 内参标定与方格实际大小无关，不需要尺子。
#include "calib.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace slam {
namespace calib {
namespace {

/** 视角上限：够了就停。 */
constexpr int kMaxViews = 40;

/** 至少这么多视角才拟合。 */
constexpr int kMinViews = 12;

/** 每步最多收几个视角；再多只是重复同一份几何。 */
constexpr int kPerStep = 3;

/** 归一化坐标超过它算画面外圈；畸变靠这些点约束。 */
constexpr float kOuterEdge = 0.7f;

struct View {
    std::vector<cv::Point2f> corners;
    int step = 0;
};

struct State {
    std::mutex mu;
    int cols = 0, rows = 0, w = 0, h = 0;
    std::vector<View> views;
    /** 九宫格累计的角点数。 */
    float cell[9] = {0};
    /** 落在画面外圈的角点数，以及角点总数。 */
    float outer = 0;
    float cellTotal = 0;
    bool fitted = false;
    /** start/reset 时递增，用来识别「probe 之后世界变了」。 */
    int gen = 0;
    /** 上一次 probe 的角点，等 commit 消费。 */
    std::vector<cv::Point2f> lastCorners;
    int lastGen = -1;
};

State& state() {
    static State s;
    return s;
}

/**
 * 取四个极值角。凸四边形的线性极值必然落在顶点上，所以与板子朝向、角点索引序无关。
 */
void extremeQuad(const std::vector<cv::Point2f>& c, float* out8) {
    static const float dir[4][2] = {{-1.f, -1.f}, {1.f, -1.f}, {1.f, 1.f}, {-1.f, 1.f}};
    for (int i = 0; i < 4; ++i) {
        int best = 0;
        float bestDot = -1e30f;
        for (size_t j = 0; j < c.size(); ++j) {
            const float d = c[j].x * dir[i][0] + c[j].y * dir[i][1];
            if (d > bestDot) {
                bestDot = d;
                best = static_cast<int>(j);
            }
        }
        out8[i * 2] = c[best].x;
        out8[i * 2 + 1] = c[best].y;
    }
}

void accumulateCoverage(State& st, const std::vector<cv::Point2f>& c) {
    for (const auto& p : c) {
        const int gx = std::max(0, std::min(2, static_cast<int>(p.x * 3 / st.w)));
        const int gy = std::max(0, std::min(2, static_cast<int>(p.y * 3 / st.h)));
        st.cell[gy * 3 + gx] += 1;
        st.cellTotal += 1;

        // 外圈按离边缘多近算，不按落在哪一格：九格会被长宽比放大。
        const float u = (2.0f * p.x - st.w) / st.w;
        const float v = (2.0f * p.y - st.h) / st.h;
        if (std::max(std::fabs(u), std::fabs(v)) >= kOuterEdge) st.outer += 1;
    }
}

int countStep(const State& st, int step) {
    int n = 0;
    for (const View& v : st.views) {
        if (v.step == step) ++n;
    }
    return n;
}

/** 一次拟合的全部产出。stdDev 是内参标准差（fx fy cx cy …）。 */
struct Fit {
    cv::Mat K, dist, stdDev;
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> perView;
    double rms = 1e9;
};

/**
 * 拟合一趟。C++ 里没有 calibrateCameraExtended 这个名字（那个别名只给其他语言绑定用），
 * 所以要调 calibrateCamera 的 11 参重载才拿得到 stdDev。
 */
bool runFit(const std::vector<View>& views, const std::vector<cv::Point3f>& obj,
            int w, int h, int flags, Fit* f) {
    if (views.size() < static_cast<size_t>(kMinViews)) return false;

    std::vector<std::vector<cv::Point3f>> objAll(views.size(), obj);
    std::vector<std::vector<cv::Point2f>> img;
    img.reserve(views.size());
    for (const View& v : views) img.push_back(v.corners);

    cv::Mat stdDevExt, perView;
    f->K = cv::Mat();
    f->dist = cv::Mat();
    f->rvecs.clear();
    f->tvecs.clear();
    f->perView.clear();

    try {
        f->rms = cv::calibrateCamera(objAll, img, cv::Size(w, h), f->K, f->dist,
                                     f->rvecs, f->tvecs, f->stdDev, stdDevExt, perView, flags);
    } catch (...) {
        return false;
    }
    if (f->K.empty() || f->dist.empty()) return false;

    const double* pv = perView.empty() ? nullptr : perView.ptr<double>();
    if (pv != nullptr) {
        const size_t n = perView.total();
        f->perView.reserve(n);
        for (size_t i = 0; i < n; ++i) f->perView.push_back(pv[i]);
    }
    return true;
}

}  // namespace

void start(int cols, int rows, int w, int h) {
    State& st = state();
    std::lock_guard<std::mutex> lk(st.mu);
    st.cols = cols;
    st.rows = rows;
    st.w = w;
    st.h = h;
    st.views.clear();
    std::fill(st.cell, st.cell + 9, 0.f);
    st.outer = 0;
    st.cellTotal = 0;
    st.fitted = false;
    ++st.gen;
    st.lastCorners.clear();
    st.lastGen = -1;
}

int probe(const unsigned char* gray, float* out8) {
    State& st = state();

    // 检测要几十到几百毫秒，不能抱锁跑：UI 每 300 ms 会调 views()/coverage()。
    int cols, rows, w, h, gen;
    {
        std::lock_guard<std::mutex> lk(st.mu);
        if (gray == nullptr || st.cols <= 0 || st.rows <= 0 || st.w <= 0 || st.h <= 0) return 0;
        if (st.fitted || static_cast<int>(st.views.size()) >= kMaxViews) return 0;
        cols = st.cols;
        rows = st.rows;
        w = st.w;
        h = st.h;
        gen = st.gen;
    }

    const cv::Mat img(h, w, CV_8UC1, const_cast<unsigned char*>(gray));
    std::vector<cv::Point2f> corners;

    // SB 抗模糊；EXHAUSTIVE 慢些，但只抽 1/5 帧。
    const int flags = cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY;
    if (!cv::findChessboardCornersSB(img, cv::Size(cols, rows), corners, flags)) {
        return 0;
    }
    if (static_cast<int>(corners.size()) != cols * rows) return 0;

    std::lock_guard<std::mutex> lk(st.mu);
    // 检测期间可能被 start()/reset() 重新开始了。
    if (st.gen != gen || st.fitted) return 0;
    st.lastCorners = corners;
    st.lastGen = gen;
    if (out8 != nullptr) extremeQuad(st.lastCorners, out8);
    return 1;
}

int commit(int step) {
    State& st = state();
    std::lock_guard<std::mutex> lk(st.mu);
    if (st.fitted) return 0;
    if (st.lastGen != st.gen || st.lastCorners.empty()) return 0;
    if (static_cast<int>(st.views.size()) >= kMaxViews) {
        st.lastCorners.clear();
        return 0;
    }
    if (countStep(st, step) >= kPerStep) {
        st.lastCorners.clear();
        return 0;
    }

    View v;
    v.corners.swap(st.lastCorners);
    v.step = step;
    st.views.push_back(std::move(v));
    accumulateCoverage(st, st.views.back().corners);
    // 一次性消费：下一次要重新 probe 才有新角点。
    st.lastCorners.clear();
    return 1;
}

int views() {
    State& st = state();
    std::lock_guard<std::mutex> lk(st.mu);
    return static_cast<int>(st.views.size());
}

void coverage(float* out10) {
    State& st = state();
    std::lock_guard<std::mutex> lk(st.mu);
    if (out10 == nullptr) return;
    const float total = st.cellTotal > 0 ? st.cellTotal : 1.f;
    for (int i = 0; i < 9; ++i) out10[i] = st.cell[i] / total;
    out10[9] = st.outer / total;
}

int finish(float* out13) {
    State& st = state();
    std::lock_guard<std::mutex> lk(st.mu);
    if (out13 == nullptr) return 0;
    // 失败也要把视角数写出去，否则调用方只能看到一个未写入的 0。
    out13[9] = static_cast<float>(st.views.size());
    out13[10] = 0.f;
    if (static_cast<int>(st.views.size()) < kMinViews) return 0;

    std::vector<cv::Point3f> obj;
    obj.reserve(st.cols * st.rows);
    for (int r = 0; r < st.rows; ++r) {
        for (int c = 0; c < st.cols; ++c) obj.emplace_back(float(c), float(r), 0.f);
    }

    const int flags = cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_ZERO_TANGENT_DIST;

    // 第一趟只为剔坏视角：一两个糊帧就能把 k1 拉偏。
    Fit first;
    if (!runFit(st.views, obj, st.w, st.h, flags, &first)) {
        printf("[CALIB] 拟合失败：%zu 个视角\n", st.views.size());
        return 0;
    }

    int dropped = 0;
    if (first.perView.size() == st.views.size()) {
        std::vector<double> sorted(first.perView);
        std::sort(sorted.begin(), sorted.end());
        const double limit = std::max(2.0 * sorted[sorted.size() / 2], 0.8);

        std::vector<View> kept;
        kept.reserve(st.views.size());
        for (size_t i = 0; i < st.views.size(); ++i) {
            if (first.perView[i] <= limit) kept.push_back(st.views[i]);
        }
        // 没超限、或剔完不够拟合的，就用这一版的结果。
        if (static_cast<int>(kept.size()) >= kMinViews && kept.size() < st.views.size()) {
            dropped = static_cast<int>(st.views.size() - kept.size());
            st.views.swap(kept);
        }
    }

    Fit f;
    if (!runFit(st.views, obj, st.w, st.h, flags, &f)) return 0;

    const double fx = f.K.at<double>(0, 0);
    const double fy = f.K.at<double>(1, 1);

    out13[0] = static_cast<float>(fx);
    out13[1] = static_cast<float>(fy);
    out13[2] = static_cast<float>(f.K.at<double>(0, 2));
    out13[3] = static_cast<float>(f.K.at<double>(1, 2));
    out13[4] = f.dist.total() > 0 ? static_cast<float>(f.dist.at<double>(0)) : 0.f;  // k1
    out13[5] = f.dist.total() > 1 ? static_cast<float>(f.dist.at<double>(1)) : 0.f;  // k2（钉 0）
    out13[6] = f.dist.total() > 2 ? static_cast<float>(f.dist.at<double>(2)) : 0.f;  // p1（零切向）
    out13[7] = f.dist.total() > 3 ? static_cast<float>(f.dist.at<double>(3)) : 0.f;  // p2
    out13[8] = static_cast<float>(f.rms);
    out13[9] = static_cast<float>(st.views.size());
    out13[10] = static_cast<float>(dropped);

    // 焦距相对标准差；取不到就填 1（= 完全不可信）。
    const bool haveSd = f.stdDev.total() >= 2 && fx > 0.0 && fy > 0.0;
    out13[11] = haveSd ? static_cast<float>(f.stdDev.at<double>(0) / fx) : 1.f;
    out13[12] = haveSd ? static_cast<float>(f.stdDev.at<double>(1) / fy) : 1.f;

    st.fitted = true;
    return 1;
}

void reset() {
    State& st = state();
    std::lock_guard<std::mutex> lk(st.mu);
    st.views.clear();
    std::fill(st.cell, st.cell + 9, 0.f);
    st.outer = 0;
    st.cellTotal = 0;
    st.fitted = false;
    ++st.gen;
    st.lastCorners.clear();
    st.lastGen = -1;
}

}  // namespace calib
}  // namespace slam
