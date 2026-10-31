// 棋盘格内参标定：检角点、按步收视角、拟合。
//
// 两步式：probe() 只检测并把 4 个外角回传，commit() 在上层判定姿势对上后入队。
// 所以这里不做去重 —— 姿态多样性由上层的目标框保证，kPerStep 负责每步的权重均衡。
//
// 全在手机本地算，视角只在内存里；接口自带锁，但拟合期间新帧会被丢掉，调用方应串行。
#pragma once

namespace slam {
namespace calib {

/** 开始一次标定。[cols]×[rows] 是**内角点**数，图像尺寸用于建覆盖网格。 */
void start(int cols, int rows, int w, int h);

/**
 * 只检测当前帧、不入队，把 4 个外角写进 [out8]（图像坐标）。
 *
 * 顺序是同 extremeQuad 的四个极值角，**不按角点索引** —— 板子翻面首角点就变，
 * 按索引取会静默拿到错位的角。[out8] 可传 nullptr。检出返回 1。
 */
int probe(const unsigned char* gray, float* out8);

/**
 * 把上一次 probe 的角点作为视角入队，标记属于第 [step] 步。
 * 没有待提交角点、本步已满、已拟合、总数已满时返回 0。
 */
int commit(int step);

/** 已接受的视角数。 */
int views();

/**
 * out 至少 10 个：前 9 个是九宫格每格的角点占比（0..1），
 * 第 10 个是落在画面外圈（最外 30%）的角点占比。
 */
void coverage(float* out10);

/**
 * 拟合。out 至少 13 个：
 * fx fy cx cy k1 k2 p1 p2 rms views dropped fxRelSd fyRelSd。
 *
 * 最后两个是焦距的相对标准差，来自 stdDeviationsIntrinsics，是「f 定没定住」的直接度量。
 * **别用 rms 代替**：板子正对时 rms 可以很低，f 却和距离共线。
 *
 * 返回 1 成功；视角不够或拟合失败返回 0。
 */
int finish(float* out13);

void reset();

}  // namespace calib
}  // namespace slam
