// 见 calib.h。
//
// 只估 fx fy cx cy k1：k2/k3/切向在 640×480 的观测范围内辨识不出来（实测 k2 连符号都会翻），
// 钉住它们反而让 fx 稳定。板子只做平移和俯仰，所以物体点就是单位间距的平面网格 ——
// 内参标定与方格实际大小无关，不需要尺子。
#include "calib.h"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace slam {
namespace calib {
namespace {

/** 视角上限：够了就停，避免内存和拟合时间随拍摄时长涨。 */
constexpr int kMaxViews = 40;

/** 至少这么多视角才拟合。 */
constexpr int kMinViews = 12;

/** 新视角和已有视角的质心距离小于这个比例、且尺寸接近，就当成重复。 */
constexpr float kSameCenterFrac = 0.06f;
constexpr float kSameSizeFrac = 0.12f;

/** 归一化坐标的绝对值超过它，就算落在画面外圈。约束畸变靠的就是这些点。 */
constexpr float kOuterEdge = 0.7f;

struct State {
    std::mutex mu;
    int cols = 0, rows = 0, w = 0, h = 0;
    std::vector<std::vector<cv::Point2f>> views;
    /** 九宫格累计的角点数。 */
    float cell[9] = {0};
    /** 落在画面外圈的角点数，以及角点总数。 */
    float outer = 0;
    float cellTotal = 0;
    bool fitted = false;
};

State& state() {
    static State s;
    return s;
}

float boxSide(const std::vector<cv::Point2f>& c) {
    float x0 = c[0].x, x1 = c[0].x, y0 = c[0].y, y1 = c[0].y;
    for (const auto& p : c) {
        x0 = std::min(x0, p.x); x1 = std::max(x1, p.x);
        y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
    }
    return std::max(x1 - x0, y1 - y0);
}

cv::Point2f center(const std::vector<cv::Point2f>& c) {
    cv::Point2f s(0, 0);
    for (const auto& p : c) s += p;
    return s * (1.0f / static_cast<float>(c.size()));
}

/** 同一个姿势重复入队会让某一块区域权重过大，拟合会被它拉偏。 */
bool tooSimilar(const State& st, const std::vector<cv::Point2f>& c) {
    const cv::Point2f cc = center(c);
    const float side = boxSide(c);
    for (const auto& v : st.views) {
        const cv::Point2f vc = center(v);
        const float vs = boxSide(v);
        const float d = std::hypot(cc.x - vc.x, cc.y - vc.y);
        if (d < kSameCenterFrac * st.w && std::fabs(side - vs) < kSameSizeFrac * vs) {
            return true;
        }
    }
    return false;
}

void accumulateCoverage(State& st, const std::vector<cv::Point2f>& c) {
    for (const auto& p : c) {
        const int gx = std::max(0, std::min(2, static_cast<int>(p.x * 3 / st.w)));
        const int gy = std::max(0, std::min(2, static_cast<int>(p.y * 3 / st.h)));
        st.cell[gy * 3 + gx] += 1;
        st.cellTotal += 1;

        // 外圈按"离画面边缘多近"算，而不是按角点落在哪一格 ——
        // 九格会被长宽比放大：横向一格只有一个板子宽，居中时必然全落中间列。
        const float u = (2.0f * p.x - st.w) / st.w;
        const float v = (2.0f * p.y - st.h) / st.h;
        if (std::max(std::fabs(u), std::fabs(v)) >= kOuterEdge) st.outer += 1;
    }
}

/** 单个视角的平均重投影误差（px）：用它判断这个视角是不是坏的。 */
float viewResidual(const std::vector<cv::Point3f>& obj,
                   const std::vector<cv::Point2f>& img,
                   const cv::Mat& rvec, const cv::Mat& tvec,
                   const cv::Mat& K, const cv::Mat& dist) {
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj, rvec, tvec, K, dist, proj);
    double sum = 0;
    for (size_t i = 0; i < proj.size() && i < img.size(); ++i) {
        sum += cv::norm(proj[i] - img[i]);
    }
    const double n = static_cast<double>(std::min(proj.size(), img.size()));
    return n > 0 ? static_cast<float>(sum / n) : 1e6f;
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
}

int addFrame(const unsigned char* gray) {
    State& st = state();

    // 检测要几十到几百毫秒，不能抱着锁跑：views()/coverage() 是 UI 线程每 300 ms
    // 调一次的，锁被占住主线程就跟着卡。
    int cols, rows, w, h;
    {
        std::lock_guard<std::mutex> lk(st.mu);
        if (gray == nullptr || st.cols <= 0 || st.rows <= 0 || st.w <= 0 || st.h <= 0) return 0;
        if (st.fitted || static_cast<int>(st.views.size()) >= kMaxViews) return 0;
        cols = st.cols;
        rows = st.rows;
        w = st.w;
        h = st.h;
    }

    const cv::Mat img(h, w, CV_8UC1, const_cast<unsigned char*>(gray));
    std::vector<cv::Point2f> corners;

    // SB 检测器比经典版抗模糊得多；EXHAUSTIVE 慢一些，但我们只抽 1/5 帧。
    const int flags = cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY;
    if (!cv::findChessboardCornersSB(img, cv::Size(cols, rows), corners, flags)) {
        return 0;
    }
    if (static_cast<int>(corners.size()) != cols * rows) return 0;

    std::lock_guard<std::mutex> lk(st.mu);
    // 检测期间可能已经攒满，或者被 start() 重新开始了。
    if (st.fitted || st.w != w || st.h != h || st.cols != cols || st.rows != rows) return 0;
    if (static_cast<int>(st.views.size()) >= kMaxViews) return 0;
    if (tooSimilar(st, corners)) return 0;

    st.views.push_back(corners);
    accumulateCoverage(st, corners);
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

int finish(float* out11) {
    State& st = state();
    std::lock_guard<std::mutex> lk(st.mu);
    if (out11 == nullptr) return 0;
    if (static_cast<int>(st.views.size()) < kMinViews) return 0;

    std::vector<cv::Point3f> obj;
    obj.reserve(st.cols * st.rows);
    for (int r = 0; r < st.rows; ++r) {
        for (int c = 0; c < st.cols; ++c) obj.emplace_back(float(c), float(r), 0.f);
    }

    const int flags = cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_ZERO_TANGENT_DIST;
    cv::Mat K, dist;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = 1e9;
    int dropped = 0;

    // 两趟：先全体拟合，剔除明显偏离的视角，再拟合一次。
    // 一两个拍糊/板子不平的视角就足以把 k1 拉偏，进而抬高整体 RMS。
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<std::vector<cv::Point3f>> objAll(st.views.size(), obj);
        K = cv::Mat();
        dist = cv::Mat();
        try {
            rms = cv::calibrateCamera(objAll, st.views, cv::Size(st.w, st.h),
                                     K, dist, rvecs, tvecs, flags);
        } catch (...) {
            return 0;
        }
        if (K.empty() || dist.empty()) return 0;
        if (pass == 1) break;

        std::vector<float> res(st.views.size());
        for (size_t i = 0; i < st.views.size(); ++i) {
            res[i] = viewResidual(obj, st.views[i], rvecs[i], tvecs[i], K, dist);
        }
        std::vector<float> sorted = res;
        std::sort(sorted.begin(), sorted.end());
        const float limit = std::max(2.0f * sorted[sorted.size() / 2], 0.8f);

        std::vector<std::vector<cv::Point2f>> kept;
        kept.reserve(st.views.size());
        for (size_t i = 0; i < st.views.size(); ++i) {
            if (res[i] <= limit) kept.push_back(st.views[i]);
        }
        // 一个都没超限，或者剔完不够拟合的，就保留这一版的拟合结果。
        if (kept.size() == st.views.size() ||
            static_cast<int>(kept.size()) < kMinViews) {
            break;
        }
        dropped = static_cast<int>(st.views.size() - kept.size());
        st.views.swap(kept);
    }

    out11[0] = K.at<double>(0, 0);
    out11[1] = K.at<double>(1, 1);
    out11[2] = K.at<double>(0, 2);
    out11[3] = K.at<double>(1, 2);
    out11[4] = dist.at<double>(0);              // k1
    out11[5] = dist.rows > 1 ? dist.at<double>(1) : 0.f;   // k2（钉 0）
    out11[6] = dist.rows > 2 ? dist.at<double>(2) : 0.f;   // p1（零切向）
    out11[7] = dist.rows > 3 ? dist.at<double>(3) : 0.f;   // p2
    out11[8] = static_cast<float>(rms);
    out11[9] = static_cast<float>(st.views.size());
    out11[10] = static_cast<float>(dropped);
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
}

}  // namespace calib
}  // namespace slam
