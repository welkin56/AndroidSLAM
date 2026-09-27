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


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Settings.h"

#include <mutex>

namespace ORB_SLAM3
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Settings;

// 原版是那个 Pangolin 窗口 + 绘制线程，整个 Pangolin 依赖就是它带来的。
// 安卓上没有 Pangolin，所以削成空壳：公有方法签名原样保留（System 和 Tracking 的调用点
// 一行都不用改），方法体全空。帧绘制本身在 FrameDrawer 里，那个类没动。
class Viewer
{
public:
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings);

    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    bool both;
};

}

#endif // VIEWER_H
