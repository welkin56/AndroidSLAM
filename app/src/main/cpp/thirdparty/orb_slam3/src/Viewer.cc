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
    // 原版在此解析 Viewer 配置并创建 Pangolin 窗口；安卓上 bUseViewer 为 false。
}

void Viewer::Run()
{
    // 空实现，原版为绘制线程主体。
}

void Viewer::RequestFinish() {}

void Viewer::RequestStop() {}

// 必须恒返回 true：Tracking::Reset 会等待 isStopped() 为真，
// 没有绘制线程时返回 false 会导致死循环。
bool Viewer::isFinished() { return true; }

bool Viewer::isStopped() { return true; }

void Viewer::Release() {}

} //namespace ORB_SLAM
