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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "GeometricCamera.h"

#include <mutex>
#include <atomic>
#include <unordered_set>

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;

class Tracking
{  

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq=std::string());

    ~Tracking();

    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);

    void GrabImuData(const IMU::Point &imuMeasurement);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetStepByStep(bool bSet);
    bool GetStepByStep();

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    void CreateMapInAtlas();
    //std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();

    //DEBUG
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder="");
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap);

    float GetImageScale();

#ifdef REGISTER_LOOP
    void RequestStop();
    bool isStopped();
    void Release();
    bool stopRequested();
#endif

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        RECENTLY_LOST=3,
        LOST=4,
        OK_KLT=5
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;

    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<Sophus::SE3f> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // add：保上面四个列表的锁。
    // `System::mMutexState` **不包括**它们（那个锁是在 `GrabImageMonocular` 返回**之后**
    // 才拿的，只保护三个快照字段）—— 上游只在 `Shutdown()` 之后才读这四张表，所以它不用锁。
    // **我们要在跑的时候从别的线程读**（2D 地图每帧拉），必须自己加一把。
    std::mutex mMutexTraj;

    /** add：清空上面那四张表（`mMutexTraj` 内）。一次运行 = 一份轨迹，`start` 时调。 */
    void ClearTrajectory();

    // frames with estimated pose
    int mTrackedFr;
    bool mbStep;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    // ---- 用户按「重定位」----
    //
    // 在 atlas 的所有图里找当前帧（不是只找当前图）。连续几次都落到同一片区域才算认出来。
    // 状态：0 空闲 / 1 采集中 / 2 没候选 / 3 有候选但没匹配上 / 4 认出来了
    //       5 前后不一致 / 6 结果已失效（地图被重置，坐标系换了）

    /** 主线程发请求。只置一个标志，干活在 ProcessRelocRequest。 */
    void RequestRelocalization();

    /** 丢掉请求并回到空闲。主线程调。 */
    void CancelRelocalization();

    /** SLAM 线程在帧边界调，每帧一次。返回当前状态码。 */
    int ProcessRelocRequest();

    /**
     * 主线程调。out 要 >= 3：
     * `[命中图的关键帧数, 它在 atlas 里按 id 排第几（1 起）, 是不是当前图自己]`。
     */
    int RelocState(int* out3);

    /**
     * 主线程调。认出来了才返回非空：给出命中的那张图，以及 `T_N<-X`
     * （目标图坐标 -> 当前图坐标），列主序。
     */
    Map* GetRelocMapAndTransform(float out16[16]);

    /**
     * 把重定位结果作废。
     *
     * **地图被重置时必须调**：`ResetActiveMap()` 清空当前图之后，初始化会用新的第一对
     * 关键帧**重新锚定原点** —— `Map::mnId` 不变，但坐标系换了。于是「另一张图 × 那个
     * 变换」算出来的位置整体偏移。回环合并用的是现场算的 Sim3，所以它不会错。
     */
    void InvalidateRelocalization(int newState);

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization
    bool mFastInit = false;


    vector<MapPoint*> GetLocalMapMPS();

    bool mbWriteStats;

#ifdef REGISTER_TIMES
    void LocalMapStats2File();
    void TrackStats2File();
    void PrintTimeStats();

    vector<double> vdRectStereo_ms;
    vector<double> vdResizeImage_ms;
    vector<double> vdORBExtract_ms;
    vector<double> vdStereoMatch_ms;
    vector<double> vdIMUInteg_ms;
    vector<double> vdPosePred_ms;
    vector<double> vdLMTrack_ms;
    vector<double> vdNewKF_ms;
    vector<double> vdTrackTotal_ms;
#endif

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    //void CreateNewMapPoints();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    bool PredictStateIMU();

    bool Relocalization();

    /**
     * 重定位的内核。F 是被重定位的那一帧（可以就是 mCurrentFrame，也可以是副本）。
     * pRefMap == nullptr 表示不限图，在所有图里找；pExcludeMap 里的图一律不收。
     * bCommit = false 时不写 mnLastRelocFrameId —— 那是「位姿刚跳变过」的开关，
     * 只问一句的探针不该碰它。
     * pReason: 1 没候选 / 2 有候选但没匹配上。
     */
    bool RelocalizeInternal(Frame& F, Map* pRefMap, Map* pExcludeMap,
                            KeyFrame** ppMatchedKF, int* pReason, bool bCommit);

    // ---- 重定位的采集状态 ----
    //
    // 请求标志和状态码跨线程（主线程发、SLAM 线程算），其余变量只有 SLAM 线程碰。

    std::atomic<bool> mbRelocRequested;
    std::atomic<int> mnRelocState;
    int mnRelocTick;        // 帧计数，每 3 帧探一次
    int mnRelocProbes;      // 探了几次
    int mnRelocHits;        // 连续命中了几次
    int mnRelocMiss;        // 连续几次探针没结果
    int mnRelocLastReason;  // 最后一次失败的原因（1 没候选 / 2 没匹配上）
    ORB_SLAM3::Map* mpRelocMap;      // 命中的图
    KeyFrame* mpRelocFirstKF;        // 第一次命中的 KF，共视判据的锚
    Sophus::SE3f mRelocLastPoseX;    // 上一次命中位姿（目标图坐标系）

    // 下面这几个要给主线程读，单独一把锁。
    std::mutex mMutexRelocOut;
    Map* mpRelocOutMap;              // 确认时固定下来，主线程拿它去枚举点云；命中当前图时为 nullptr
    int mnRelocMapKFs;
    int mnRelocMapRank;
    int mnRelocOutSame;              // 认出来的是不是当前图自己
    Sophus::SE3f mRelocT_NX;         // 目标图坐标 -> 当前图坐标

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();

    bool mbMapUpdated;

    // Imu preintegration from last frame
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    bool bStepByStep;

    //Atlas
    Atlas* mpAtlas;

    //Calibration matrix
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;
    float mImageScale;

    float mImuFreq;
    double mImuPer;
    bool mInsertKFsLost;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    double mTimeStampLost;
    double time_recently_lost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;

    //Motion Model
    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    //int nMapChangeIndex;

    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera* mpCamera, *mpCamera2;

    int initID, lastID;

    Sophus::SE3f mTlr;

    void newParameterLoader(Settings* settings);

#ifdef REGISTER_LOOP
    bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif

public:
    cv::Mat mImRight;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
