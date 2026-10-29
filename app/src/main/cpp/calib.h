// 棋盘格内参标定：检角点、攒视角、拟合。
//
// 只在手机本地算（OpenCV calib3d），不依赖 PC，也不落盘 —— 视角全在内存里，
// 算完由 Kotlin 侧决定存不存。所有接口自带锁，可以从中任意线程调；
// 但拟合期间新帧会被丢掉，所以调用方仍然应该串行用。
#pragma once

namespace slam {
namespace calib {

/** 开始一次标定。[cols]×[rows] 是**内角点**数，图像尺寸用于建覆盖网格。 */
void start(int cols, int rows, int w, int h);

/**
 * 喂一帧紧凑灰度（w×h，无行填充）。
 * 收下返回 1；没检出、或者和已有视角太像（同一个姿势重复）返回 0。
 */
int addFrame(const unsigned char* gray);

/** 已接受的视角数。 */
int views();

/**
 * out 至少 10 个：前 9 个是九宫格每格的角点占比（0..1），
 * 第 10 个是落在画面外圈（最外 30%）的角点占比 —— 判据用这个。
 */
void coverage(float* out10);

/**
 * 拟合。out 至少 11 个：fx fy cx cy k1 k2 p1 p2 rms views dropped。
 * [dropped] 是被当成坏视角剔掉的个数。
 * 返回 1 成功；视角不够或拟合失败返回 0。
 */
int finish(float* out11);

void reset();

}  // namespace calib
}  // namespace slam
