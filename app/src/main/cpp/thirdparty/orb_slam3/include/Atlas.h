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

#ifndef ATLAS_H
#define ATLAS_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>


namespace ORB_SLAM3
{
class Viewer;
class Map;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;

//BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")
//BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

class Atlas
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar.template register_type<Pinhole>();
        ar.template register_type<KannalaBrandt8>();

        // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        //ar & mspMaps;
        ar & mvpBackupMaps;
        ar & mvpCameras;
        // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
        ar & Map::nNextId;
        ar & Frame::nNextId;
        ar & KeyFrame::nNextId;
        ar & MapPoint::nNextId;
        ar & GeometricCamera::nNextId;
        ar & mnLastInitKFidMap;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Atlas();
    Atlas(int initKFid); // When its initialization the first map is created
    ~Atlas();

    void CreateNewMap();
    void ChangeMap(Map* pMap);

    unsigned long int GetLastInitKFid();

    void SetViewer(Viewer* pViewer);

    // Method for change components in the current map
    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    //void EraseMapPoint(MapPoint* pMP);
    //void EraseKeyFrame(KeyFrame* pKF);

    GeometricCamera* AddCamera(GeometricCamera* pCam);
    std::vector<GeometricCamera*> GetAllCameras();

    /* All methods without Map pointer work on current map */
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    long unsigned int MapPointsInMap();
    long unsigned KeyFramesInMap();

    // Method for get data in current map
    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    vector<Map*> GetAllMaps();

    int CountMaps();

    void clearMap();

    void clearAtlas();

    Map* GetCurrentMap();

    // add：只读当前图，不会因坏图自旋，可从任意线程调用。
    Map* PeekCurrentMap();

    void SetMapBad(Map* pMap);
    void RemoveBadMaps();

    bool isInertial();
    void SetInertialSensor();
    void SetImuInitialized();
    bool isImuInitialized();

    // Function for garantee the correction of serialization of this object
    void PreSave();
    void PostLoad();

    /**
     * add：反序列化后、PostLoad 前自检：每张图的备份引用必须落在本图内。
     * 不一致时返回 false，由调用方判定载入失败。
     */
    bool CheckBackupConsistency();

    // add：把另一个 Atlas 的内容搬进本对象，不替换指针。
    // 搬完后 p 只剩空壳，可以 delete。
    void AdoptFrom(Atlas* p);

    // ---------------- add：合并链（仅内存）----------------
    //
    // 合并只改变坐标系、不改变图的身份。磁盘轨迹需要按记录的变换更新。
    // 链中只保留尚未随 .osa 落盘的合并；PostLoad()/ReloadAtlasFrom() 会清空。

    /** 一次合并。T_on 把幸存图合并前的坐标变换到吸收图坐标系（p' = s*R*p + t）。 */
    struct MapMergeRecord
    {
        unsigned long survivor, absorbed;
        float s;
        float q[4];   // T_on 旋转 x y z w
        float t[3];   // T_on 平移
    };

    /** 记一次合并（LoopClosing 线程）。 */
    void RecordMerge(unsigned long survivor, unsigned long absorbed,
                     const Sophus::SE3f& T_on, float s);

    /**
     * 读合并链（主线程），每条 10 个 float：
     * survivor absorbed s qx qy qz qw tx ty tz，按发生顺序；装不下时少给。
     */
    int GetMergeRecords(float* out, int cap);

    /** 丢掉最前面的 n 条（已随 .osa 落盘），返回实际条数。 */
    int DropMergeRecords(int n);

    void ClearMergeRecords();

    /** 返回 id 最大且有关键帧的图；没有时返回 -1。 */
    long long GetBestMapId();

    /**
     * 返回 displayId 所在合并族中每个文件夹 id 到当前坐标系的变换，
     * 每条 9 个 float：id, s, qx, qy, qz, qw, tx, ty, tz。
     * displayId 固定排第一；无合并时为单位变换；装不下时少给。
     */
    int GetDisplayTransforms(long long displayId, float* out, int cap);

    map<long unsigned int, KeyFrame*> GetAtlasKeyframes();

    void SetKeyFrameDababase(KeyFrameDatabase* pKFDB);
    KeyFrameDatabase* GetKeyFrameDatabase();

    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    ORBVocabulary* GetORBVocabulary();

    long unsigned int GetNumLivedKF();

    long unsigned int GetNumLivedMP();

protected:

    std::set<Map*> mspMaps;
    std::set<Map*> mspBadMaps;
    // Its necessary change the container from set to vector because libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
    std::vector<Map*> mvpBackupMaps;

    Map* mpCurrentMap;

    std::vector<GeometricCamera*> mvpCameras;

    unsigned long int mnLastInitKFidMap;

    // add：合并链，进程内按发生顺序保存，使用独立锁。
    std::vector<MapMergeRecord> mvMerges;
    std::mutex mMutexMerge;

    Viewer* mpViewer;
    bool mHasViewer;

    // Class references for the map reconstruction from the save file
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    // Mutex
    std::mutex mMutexAtlas;


}; // class Atlas

} // namespace ORB_SLAM3

#endif // ATLAS_H
