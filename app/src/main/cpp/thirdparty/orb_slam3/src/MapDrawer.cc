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

#include "MapDrawer.h"

namespace ORB_SLAM3
{


MapDrawer::MapDrawer(Atlas* pAtlas, const string &/*strSettingPath*/, Settings* /*settings*/):mpAtlas(pAtlas)
{
    // 原版在这里解析 yaml 的 Viewer 段：关键帧大小、点大小、视点位置……全是 Pangolin 的绘图参数。
}

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &/*Tcw*/)
{
    // 原版把 Tcw.inverse() 锁进 mCameraPose，供 Viewer 取出来画相机位置。
    // 没有 Viewer 之后这个值没人读，于是连求逆和加锁都省掉 —— 它在 Tracking 里是每帧都调的。
}

} //namespace ORB_SLAM
