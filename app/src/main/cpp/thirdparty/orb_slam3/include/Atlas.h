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

    // add：只读，**不会**睡眠等坏图。给「从别的线程问一句」用 ——
    // `GetCurrentMap()` 在坏图窗口里是 `while(IsBad()) usleep(3000)`，还抱着 `mMutexAtlas`。
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
     * add：反序列化之后、`PostLoad()` 之前自检 —— 每张图的备份引用是不是都落在
     * **同一张图**里。不一致的文件 `PostLoad()` 会往容器里塞 nullptr（旧的文件可能
     * 有这种，见 `Atlas::PreSave` 的注释），所以宁可当场判载入失败，也别让它崩。
     */
    bool CheckBackupConsistency();

    // add:把另一个 Atlas 反序列化出来的内容搬进来。
    // Tracking / LocalMapping / LoopClosing 都攥着 `System::mpAtlas` —— 换指针就得让它们
    // 全部改指向，所以载进现有的对象、只搬内容。搬完 `p` 只剩空壳，可以 delete。
    void AdoptFrom(Atlas* p);

    // ---------------- add：合并链（仅内存）----------------
    //
    // 合并只改**坐标系**、不改图的身份：活着的是当前图，被吸收的图 0 关键帧、
    // 存盘时会被 `PreSave()` 剔掉。磁盘上的轨迹要跟着坐标系走，所以得把变换记下来。
    //
    // 链里留的是**还没写到磁盘上的**合并：存图成功后由 `System::DropMergeRecords()`
    // 丢掉已经随 `.osa` 落盘的那几条。`PostLoad()` / `ReloadAtlasFrom()` 整条清空。

    /** 一次合并。`T_on` 把幸存图**合并前**的坐标搬到吸收图的坐标里（`p' = s·R·p + t`）。 */
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
     * 读合并链（主线程）：每条 10 个 float `survivor absorbed s qx qy qz qw tx ty tz`，
     * 按发生顺序。**装不下就少给，返回实际写了几条。**
     */
    int GetMergeRecords(float* out, int cap);

    /** 丢掉链里最前面的 n 条（已经随 `.osa` 落盘的）。返回实际丢了几条。 */
    int DropMergeRecords(int n);

    void ClearMergeRecords();

    /** 图集里 id 最大且有关键帧的那张图（0 关键帧的不算）；一张都没有返回 -1。 */
    long long GetBestMapId();

    /**
     * `displayId` **那个合并族里**每个文件夹 id 到它此刻所在坐标系的变换，每条 9 个
     * float：`id, s, qx, qy, qz, qw, tx, ty, tz`（`p' = s·R·p + t`）。
     *
     * `displayId` 那条**固定排第一**；没有合并时它就是单位变换。被吸收的图原样，
     * 幸存图那一族再乘 `T_on` —— 多跳按链的顺序逐条累积。别的族（换过 current map
     * 之前留下的合并）不输出。装不下就少给。
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

    // add：合并链。进程内、按发生顺序。独立锁 —— 读写都不需要碰 `mMutexAtlas`。
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
