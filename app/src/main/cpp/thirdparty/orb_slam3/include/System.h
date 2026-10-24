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


#ifndef SYSTEM_H
#define SYSTEM_H


#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"


namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string());

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // add:`ResetActiveMap` 原地清空当前图，`Map::mnId` 不变 ——
    // 所以 KF 数掉到 0 是「重置过」唯一的外部信号。
    unsigned long KeyFramesInMap() { return mpAtlas->KeyFramesInMap(); }

    // add:当前图的**全部地图点**（`Atlas::GetAllMapPoints()` 的转发）。给 2D 地图的
    // 点云快照用；`isBad()` 的过滤、抽稀、取坐标都在调用方做。
    //
    //  **只能从 tracking 线程调** —— 它直接读 `mpCurrentMap`，而唯一会读到失效指针的
    //    窗口在 `Tracking::Reset()` 里（`clearAtlas()` 和 `CreateNewMap()` 之间），
    //    那也正是跑在 tracking 线程上的。
    std::vector<MapPoint*> GetAllMapPoints();

    /**
     * add:把当前 atlas **另存到** `<path>.osa`（`path` **不含**扩展名）。
     *
     * 不跟 `LoadAtlas` 一样走 yaml：yaml 是构造时读一次就定死的，而文件名要现取。
     * 只在 SLAM 停止之后调 —— 它序列化整张 atlas。
     */
    void SaveAtlasTo(const std::string& path);

    /**
     * add:在**进程内**把 atlas 换回 `<path>.osa` 的那份状态（`path` 空 = 清成一张空图）。
     *
     * **只能在 SLAM 线程、且没有帧在动的时候调** —— 它走 `Tracking::Reset()`，
     * 那里面会阻塞等 LocalMapping / LoopClosing 应答；主线程调就是卡死界面。
     * 读失败时留下一张空图（系统还能继续跑），返回 false。
     */
    bool ReloadAtlasFrom(const std::string& path);

    /** add：atlas 里现在有几张图 / 一共多少关键帧 / 多少地图点。给界面显示用。 */
    void GetAtlasStats(int* maps, int* kfs, int* mps);

    /**
     * add：atlas 里**有关键帧**的图（按 id 升序）。三条出参一一对应，返回实际写了几张。
     * 给 2D 地图查看器的选图列表用；0 关键帧的空图不返回。
     */
    int GetMapList(long long* ids, int* kfs, int* mps, int cap);

    /**
     * add：**任意一张图**的点云（该图自己的坐标系），stride 抽稀到 `cap/3` 个点，
     * 写 `3n` 个 `x y z`。找不到这张图返回 0。
     *
     * 会拿 `mMutexMap` 拷一份指针表 —— 只给「选图 / 刷新」这种低频动作调，别每帧调。
     */
    int GetMapCloud(long long mapId, float* out, int cap);

    /**
     * add:这一趟的**相机轨迹**（地图坐标系的 `Twc`），每点 8 个：`t x y z qx qy qz qw`。
     *
     * `cap` = `out` 能装多少 float ⟹ 最多写 `cap/8` 个点。
     * **超过就按 stride 抽稀**（覆盖全程、恒含最新点），**不是截断** ——
     * 所以要「取全」就传一个 `≥ 8 × 总帧数` 的 `cap`。
     *（和 `getPinned` 那个 `cap` 的语义**相反**：那个装不下就少给、靠扩容补全。）
     *
     * 数据来自 `mpTracker` 那份「相对参考关键帧」的列表，读时用关键帧当前的位姿重算
     * ⟹ 回环/BA 改过地图之后仍然一致。和 `SaveTrajectoryEuRoC` 有两处故意不同，见 .cc。
     */
    int GetFrameTrajectory(float* out, int cap);

    /** 上游那份轨迹表里有多少帧（含跟丢的）。给状态栏的计数用 —— 不抽稀。 */
    int GetFrameTrajectorySize();

    /**
     * add：这一趟**最后一段**轨迹属于哪张图（没有 → -1）。
     *
     * ⚠️ 判据必须和 `GetFrameTrajectory()` 完全一致（最后那个非空 reference 属于哪张图），
     * 否则导出的轨迹和它落进的文件夹会对不上。
     */
    long long GetTrajectoryMapId();

    /**
     * add：本进程的合并链，每条 10 个 float
     * `survivor absorbed s qx qy qz qw tx ty tz`，按发生顺序。**装不下就少给。**
     */
    int GetMergeRecords(float* out, int cap);

    /** add：丢掉链里最前面的 n 条（已经随 `.osa` 落盘的）。返回实际丢了几条。 */
    int DropMergeRecords(int n);

    /**
     * add：界面显示用的图 id —— 当前轨迹所属的图；没有轨迹（刚载入 / Reset 之后）
     * 就取图集里 id 最大且有关键帧的那张。没有 → -1。
     */
    long long GetDisplayMapId();

    /**
     * add：`GetDisplayMapId()` 那张图所在坐标系里，每个文件夹 id 的累计变换。
     * 每条 9 个：`id s qx qy qz qw tx ty tz`，**第一条固定是 displayMapId**。
     */
    int GetDisplayTransforms(float* out, int cap);

    // ---------------- add：路径点 ----------------
    //
    // 「记一个路径点」= 记下**此刻的相机位姿 + 一个名字**。它和轨迹用同一套存法：
    // 锚在一个关键帧上、存相对位姿，读的时候现乘 —— 于是 BA / 回环 / 合并改了地图之后，
    // 路径点自动跟着走，不用任何合并钩子。锚点被剔了就沿 `mTcp` 爬到活着的父帧；
    // 只有 `GetMap() == nullptr`（`Map::clear()`，即地图被重置）才算失效。

    /**
     * add：在当前相机位置记一个路径点（**SLAM 线程调** —— 要读 `mCurrentFrame`）。
     * 当前图里还没有关键帧时返回 false。
     */
    bool AddWaypoint(const std::string& name);
    /** 最后一次 `AddWaypoint` 失败的原因（空串 = 上次成功）。 */
    std::string GetWaypointError();

    /** add：改名 / 删除 / 清空。**任意线程** —— 只碰字符串和容器。 */
    void RenameWaypoint(int id, const std::string& name);
    void DeleteWaypoint(int id);
    void ClearWaypoints();

    /** add：路径点条数（任意线程）。 */
    int GetWaypointCount();

    /** add：下一个会分配的 id（= 默认名字里的编号）。任意线程。 */
    int GetNextWaypointId();

    /** add：第 `i` 条的 id 与名字（任意线程）。找不到返回空串。 */
    std::string GetWaypointName(int id);

    /**
     * add：读路径点的世界坐标（**主线程调**）。
     *
     * `pos` 每点 3 个 float（`x y z`），`meta` 每点 4 个：
     * `id, mapId, frameId, flags`（bit0 = 坐标有效 / bit1 = 在当前图）。
     *
     * ⚠️ 容量契约和 `getPinned` 一样：**装不下就少给，返回实际写了几个**，
     * 调用方看到写满就扩容 —— 不能返回真实条数，那调用方会读到没写过的内存。
     */
    int GetWaypoints(float* pos, int posCap, long long* meta, int metaCap);

    /** add：变一次就加一。给界面判断「要不要重建那层叠加」用。 */
    long long GetWaypointRevision();

    /** add：存 / 读 sidecar（`<path>.waypoints.tsv`）。**SLAM 线程调**；空路径 = 只清空。 */
    bool SaveWaypoints(const std::string& path);
    bool LoadWaypoints(const std::string& path);

    /** add：清空那份轨迹表。**一次运行 = 一份轨迹**，`start` 时调（否则会跨 run 累加）。 */
    void ClearFrameTrajectory();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();
    bool isShutDown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    void SaveTrajectoryEuRoC(const string &filename);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

    void SaveTrajectoryEuRoC(const string &filename, Map* pMap);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap);

    // Save data used for initialization debug
    void SaveDebugData(const int &iniIdx);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

    float GetImageScale();

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

private:

    void SaveAtlas(int type);
    bool LoadAtlas(int type);

    // add:同 LoadAtlas，但路径现给，而且**读进现有的 Atlas**（不换指针）。
    bool LoadAtlasInto(const std::string& path, int type);

    string CalculateCheckSum(string filename, int type);

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //Map* mpMap;
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Shutdown flag
    bool mbShutDown;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    //
    string mStrLoadAtlasFromFile;
    string mStrSaveAtlasToFile;

    string mStrVocabularyFilePath;

    // add：路径点。**只在 `System.cc` 里定义具体类型** —— 这个头文件只用前置声明，
    // 免得为了一个 `Sophus::SE3f` 成员把它们拖进来。
    struct Waypoint;
    std::vector<Waypoint*> mvpWaypoints;
    std::mutex mMutexWaypoints;
    long long mnWaypointRevision;
    int mnNextWaypointId;
    /** 最后一次 `AddWaypoint` 失败的原因。空串 = 上次是成功的。 */
    std::string mstrWaypointError;

    Settings* settings_;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
