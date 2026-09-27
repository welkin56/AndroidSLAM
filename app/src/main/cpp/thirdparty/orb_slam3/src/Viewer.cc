/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"

namespace ORB_SLAM3
{

Viewer::Viewer(System* /*pSystem*/, FrameDrawer* /*pFrameDrawer*/, MapDrawer* /*pMapDrawer*/,
               Tracking* /*pTracking*/, const string &/*strSettingPath*/, Settings* /*settings*/):
    both(false)
{
    // 原版在这里解析 yaml 的 Viewer 段、建 Pangolin 窗口、算出绘图用的图像尺寸。
    // bUseViewer=false 时（安卓这条线）根本不会构造到它。
}

void Viewer::Run()
{
    // 空实现。原版是这个绘制线程：取地图、画点、画关键帧、imshow 当前帧。
}

void Viewer::RequestFinish() {}

void Viewer::RequestStop() {}

// 这两个恒为 true 是刻意的，不是省事 —— Tracking::Reset 里有
//     mpViewer->RequestStop();  while(!mpViewer->isStopped()) usleep(3000);
// 原版靠绘制线程把 mbStopped 翻成 true 才能退出这个循环。线程没了之后，
// 只要这里返回一次 false，那里就是一个永远退不出来的死循环。
bool Viewer::isFinished() { return true; }

bool Viewer::isStopped() { return true; }

void Viewer::Release() {}

} //namespace ORB_SLAM
