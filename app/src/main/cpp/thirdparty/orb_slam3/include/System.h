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

    /**
     * add：直接设定相机内参（畸变取 k1..p2，k3 固定为 0）。
     * 只能在 nativeStart() 之前、没有任何帧在跑的时候调用：
     * LocalMapping / LoopClosing 读这些量时没有加锁。
     */
    void SetCameraIntrinsics(float fx, float fy, float cx, float cy,
                             float k1, float k2, float p1, float p2);

    /**
     * add：替换 IMU 噪声密度（连续量，每 √Hz，与 yaml 同单位）。
     * 与 SetCameraIntrinsics 同一前置条件：只在没有帧在跑时调用。
     */
    void SetImuNoise(float ng, float na, float ngw, float naw);

    // add：ResetActiveMap 原地清空当前图，Map::mnId 不变，
    // 因此关键帧数降到 0 是「发生过重置」的唯一外部信号。
    unsigned long KeyFramesInMap() { return mpAtlas->KeyFramesInMap(); }

    // add：返回当前图的全部地图点，供 2D 点云快照使用。
    // 只能从 tracking 线程调用：它直接读 mpCurrentMap。
    std::vector<MapPoint*> GetAllMapPoints();

    /**
     * add：把当前 atlas 另存到 path + ".osa"（path 不含扩展名）。
     * 文件名需要临时指定，因此不走 yaml；只在 SLAM 停止后调用。
     */
    void SaveAtlasTo(const std::string& path);

    /**
     * add：在进程内把 atlas 换回 path + ".osa" 的状态（path 为空表示空图）。
     * 只能在 SLAM 线程且没有帧处理时调用；失败时留下空图并返回 false。
     */
    bool ReloadAtlasFrom(const std::string& path);

    /** add：返回 atlas 的图数、关键帧数和地图点数。 */
    void GetAtlasStats(int* maps, int* kfs, int* mps);

    /**
     * add：返回 atlas 中有关键帧的图，按 id 升序；三组出参一一对应。
     */
    int GetMapList(long long* ids, int* kfs, int* mps, int cap);

    /**
     * add：返回指定图的点云（该图自身坐标系），按 stride 抽稀到 cap/3 点，
     * 写入 3n 个 x y z；找不到该图时返回 0。仅用于低频操作。
     */
    int GetMapCloud(long long mapId, float* out, int cap);

    /**
     * add：返回本趟相机轨迹（地图坐标系的 Twc），每点 8 个：t x y z qx qy qz qw。
     * 超过 cap/8 点时按 stride 抽稀，覆盖全程并包含最新点。
     * 数据由相对参考关键帧的位姿实时合成，BA 或回环后仍保持一致。
     */
    int GetFrameTrajectory(float* out, int cap);

    /** 上游轨迹表中的帧数（含跟丢的帧），不抽稀。 */
    int GetFrameTrajectorySize();

    /**
     * add：返回本趟最后一段轨迹所属的图，没有时返回 -1。
     * 判据必须与 GetFrameTrajectory() 一致。
     */
    long long GetTrajectoryMapId();

    /**
     * add：返回本进程的合并链，每条 10 个 float：
     * survivor absorbed s qx qy qz qw tx ty tz，按发生顺序；装不下时少给。
     */
    int GetMergeRecords(float* out, int cap);

    /** add：丢掉最前面的 n 条合并记录（已随 .osa 落盘），返回实际条数。 */
    int DropMergeRecords(int n);

    /**
     * add：返回界面显示使用的图 id：当前轨迹所属的图；没有轨迹时取 id 最大且
     * 有关键帧的图；都没有时返回 -1。
     */
    long long GetDisplayMapId();

    /**
     * add：返回显示图所在坐标系中每个文件夹 id 的累计变换。
     * 每条 9 个 float：id s qx qy qz qw tx ty tz；第一条固定为 displayMapId。
     */
    int GetDisplayTransforms(float* out, int cap);

    // ---------------- add：路径点 ----------------
    //
    // 路径点保存当前相机位姿和名字，锚定在关键帧上并保存相对位姿。
    // 地图变化后自动跟随；锚点失效时沿 mTcp 上溯到存活帧。

    /**
     * add：在当前相机位置记录路径点（SLAM 线程调用）。
     * 当前图没有关键帧时返回 false。
     */
    bool AddWaypoint(const std::string& name);
    /** 最后一次 AddWaypoint 失败的原因；成功时为空串。 */
    std::string GetWaypointError();

    /** add：改名、删除或清空路径点，任意线程可调。 */
    void RenameWaypoint(int id, const std::string& name);
    void DeleteWaypoint(int id);
    void ClearWaypoints();

    /** add：路径点条数，任意线程可调。 */
    int GetWaypointCount();

    /** add：返回下一个路径点 id，任意线程可调。 */
    int GetNextWaypointId();

    /** add：返回指定 id 的路径点名字，任意线程可调；找不到时返回空串。 */
    std::string GetWaypointName(int id);

    /**
     * add：读取路径点世界坐标（主线程调用）。
     * pos 每点 3 个 float；meta 每点 4 个：id, mapId, frameId, flags
     * （bit0 坐标有效 / bit1 在当前图）。装不下时少给，返回实际条数。
     */
    int GetWaypoints(float* pos, int posCap, long long* meta, int metaCap);

    /** add：路径点每变化一次加一，供界面判断是否重建叠加层。 */
    long long GetWaypointRevision();

    /** add：保存或读取 sidecar（path.waypoints.tsv），SLAM 线程调用；空路径表示只清空。 */
    bool SaveWaypoints(const std::string& path);
    bool LoadWaypoints(const std::string& path);

    /** add：清空轨迹表。一次运行对应一份轨迹，start 时调用。 */
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

    // add：与 LoadAtlas 相同，但路径由参数指定，并读入现有 Atlas（不替换指针）。
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

    // add：路径点。具体类型只在 System.cc 中定义，这里使用前置声明。
    struct Waypoint;
    std::vector<Waypoint*> mvpWaypoints;
    std::mutex mMutexWaypoints;
    long long mnWaypointRevision;
    int mnNextWaypointId;
    /** 最后一次 AddWaypoint 失败的原因；成功时为空串。 */
    std::string mstrWaypointError;

    Settings* settings_;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
