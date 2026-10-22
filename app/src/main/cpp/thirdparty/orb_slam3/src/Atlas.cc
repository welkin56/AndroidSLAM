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

#include "Atlas.h"
#include "Viewer.h"

#include <algorithm> // std::min（GetMergeRecords）
#include <map>       // 文件夹 id → 累计变换（GetDisplayTransforms）
#include <utility>   // std::move（AdoptFrom）

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3
{

Atlas::Atlas(){
    mpCurrentMap = static_cast<Map*>(NULL);
}

Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
{
    mpCurrentMap = static_cast<Map*>(NULL);
    CreateNewMap();
}

Atlas::~Atlas()
{
    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        Map* pMi = *it;

        if(pMi)
        {
            delete pMi;
            pMi = static_cast<Map*>(NULL);

            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}

void Atlas::CreateNewMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Creation of new map with id: " << Map::nNextId << endl;
    if(mpCurrentMap){
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        cout << "Stored map with ID: " << mpCurrentMap->GetId() << endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Change to map with id: " << pMap->GetId() << endl;
    if(mpCurrentMap){
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame* pKF)
{
    Map* pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint* pMP)
{
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
{
    //Check if the camera already exists
    bool bAlreadyInMap = false;
    int index_cam = -1;
    for(size_t i=0; i < mvpCameras.size(); ++i)
    {
        GeometricCamera* pCam_i = mvpCameras[i];
        if(!pCam) std::cout << "Not pCam" << std::endl;
        if(!pCam_i) std::cout << "Not pCam_i" << std::endl;
        if(pCam->GetType() != pCam_i->GetType())
            continue;

        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            if(((Pinhole*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
        else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            if(((KannalaBrandt8*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
    }

    if(bAlreadyInMap)
    {
        return mvpCameras[index_cam];
    }
    else{
        mvpCameras.push_back(pCam);
        return pCam;
    }
}

std::vector<GeometricCamera*> Atlas::GetAllCameras()
{
    return mvpCameras;
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

vector<Map*> Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap = static_cast<Map*>(NULL);
    mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(!mpCurrentMap)
        CreateNewMap();
    while(mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

Map* Atlas::PeekCurrentMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap;
}

void Atlas::SetMapBad(Map* pMap)
{
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps()
{
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/
    mspBadMaps.clear();
}

bool Atlas::isInertial()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

void Atlas::PreSave()
{
    if(mpCurrentMap){
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum
    }

    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };

    // ⚠️ 先清空再重建。同一进程里连续存图（跑一趟→存→再跑→存）时，这里会留着
    //    上一次的列表：被合并掉的图还挂着**上一次**的备份数组，而里面的 KF/MP
    //    早已搬到别的图去了 —— 写出去的文件自相矛盾，载入时 `PostLoad` 找不到 id。
    //    空图 / 坏图也一并挡在列表外（它们没有意义，还可能是上面那种旧数组）。
    mvpBackupMaps.clear();
    vector<Map*> vMaps(mspMaps.begin(), mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());

    std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
    for(Map* pMi : vMaps)
    {
        if(!pMi || pMi->IsBad())
            continue;

        if(pMi->GetAllKeyFrames().size() == 0) {
            // Empty map, erase before of save it.
            SetMapBad(pMi);
            continue;
        }
        mvpBackupMaps.push_back(pMi);
        pMi->PreSave(spCams);
    }
    RemoveBadMaps();
}

bool Atlas::CheckBackupConsistency()
{
    for (Map* pMap : mvpBackupMaps)
        if (pMap && !pMap->CheckBackupConsistency())
            return false;
    return true;
}

void Atlas::PostLoad()
{
    map<unsigned int,GeometricCamera*> mpCams;
    for(GeometricCamera* pCam : mvpCameras)
    {
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    for(Map* pMi : mvpBackupMaps)
    {
        mspMaps.insert(pMi);
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }
    mvpBackupMaps.clear();

    // 链属于**被换掉的**那个 atlas：读进来的图各自是独立坐标系，旧链一条都不能留。
    ClearMergeRecords();
}

void Atlas::AdoptFrom(Atlas* p)
{
    if(!p)
        return;

    // 只搬 `serialize` 装的那几个字段。`mspMaps` / `mpCurrentMap` 由调用方在
    // `PostLoad()` + `CreateNewMap()` 里重建（这里动会踩到 reset 刚建的那张空图）。
    // 静态 ID 计数器是 `serialize` 直接写的，已经在全局生效，不用搬。
    unique_lock<mutex> lock(mMutexAtlas);
    mvpBackupMaps = std::move(p->mvpBackupMaps);
    mvpCameras = std::move(p->mvpCameras);
    mnLastInitKFidMap = p->mnLastInitKFidMap;
}

void Atlas::RecordMerge(unsigned long survivor, unsigned long absorbed,
                        const Sophus::SE3f& T_on, float s)
{
    MapMergeRecord r;
    r.survivor = survivor;
    r.absorbed = absorbed;
    r.s = s;

    const Eigen::Quaternionf q = T_on.unit_quaternion();
    r.q[0] = q.x(); r.q[1] = q.y(); r.q[2] = q.z(); r.q[3] = q.w();
    const Eigen::Vector3f t = T_on.translation();
    r.t[0] = t.x(); r.t[1] = t.y(); r.t[2] = t.z();

    unique_lock<mutex> lock(mMutexMerge);
    mvMerges.push_back(r);
}

int Atlas::GetMergeRecords(float* out, int cap)
{
    if (out == nullptr || cap < 10)
        return 0;

    unique_lock<mutex> lock(mMutexMerge);
    const int n = std::min(static_cast<int>(mvMerges.size()), cap / 10);
    for (int i = 0; i < n; ++i)
    {
        const MapMergeRecord& r = mvMerges[static_cast<size_t>(i)];
        float* o = out + 10 * i;
        // id 走 float 传：一次会话里的 map id 远小于 2^24，精度够。
        o[0] = static_cast<float>(r.survivor);
        o[1] = static_cast<float>(r.absorbed);
        o[2] = r.s;
        o[3] = r.q[0]; o[4] = r.q[1]; o[5] = r.q[2]; o[6] = r.q[3];
        o[7] = r.t[0]; o[8] = r.t[1]; o[9] = r.t[2];
    }
    return n;
}

int Atlas::DropMergeRecords(int n)
{
    unique_lock<mutex> lock(mMutexMerge);
    const int k = std::min(std::max(n, 0), static_cast<int>(mvMerges.size()));
    if (k > 0)
        mvMerges.erase(mvMerges.begin(), mvMerges.begin() + k);
    return k;
}

void Atlas::ClearMergeRecords()
{
    unique_lock<mutex> lock(mMutexMerge);
    mvMerges.clear();
}

long long Atlas::GetBestMapId()
{
    long long best = -1;
    for (Map* pMap : GetAllMaps())
    {
        if (pMap == nullptr || pMap->KeyFramesInMap() == 0)
            continue;
        const long long id = static_cast<long long>(pMap->GetId());
        if (id > best)
            best = id;
    }
    return best;
}

int Atlas::GetDisplayTransforms(long long displayId, float* out, int cap)
{
    if (out == nullptr || cap < 9 || displayId < 0)
        return 0;

    // 只用 POD，不让 Eigen 类型进容器（对对齐有要求）。
    struct Xform
    {
        float s;
        float q[4];           // x y z w
        float t[3];
        unsigned long owner;  // 已经并进哪张图的那一族
    };

    unique_lock<mutex> lock(mMutexMerge);

    const unsigned long display = static_cast<unsigned long>(displayId);
    std::map<unsigned long, Xform> m;
    auto ensure = [&m](unsigned long id) -> Xform& {
        auto it = m.find(id);
        if (it == m.end())
            it = m.emplace(id, Xform{1.f, {0.f, 0.f, 0.f, 1.f}, {0.f, 0.f, 0.f}, id}).first;
        return it->second;
    };
    ensure(display);

    for (const MapMergeRecord& r : mvMerges)
    {
        ensure(r.survivor);
        ensure(r.absorbed);

        const Eigen::Quaternionf q2(r.q[3], r.q[0], r.q[1], r.q[2]);
        const Eigen::Vector3f t2(r.t[0], r.t[1], r.t[2]);

        // 幸存图那一族整体再乘一次 T_on：p' = s2·R2·(s1·R1·p + t1) + t2。
        for (auto& kv : m)
        {
            if (kv.second.owner != r.survivor)
                continue;
            Xform& x = kv.second;
            const Eigen::Quaternionf qx(x.q[3], x.q[0], x.q[1], x.q[2]);
            const Eigen::Vector3f t =
                r.s * (q2 * Eigen::Vector3f(x.t[0], x.t[1], x.t[2])) + t2;
            const Eigen::Quaternionf q = (q2 * qx).normalized();
            x.s = r.s * x.s;
            x.q[0] = q.x(); x.q[1] = q.y(); x.q[2] = q.z(); x.q[3] = q.w();
            x.t[0] = t.x(); x.t[1] = t.y(); x.t[2] = t.z();
        }

        // 被吸收图连同它已有的族并进幸存图；它自己的变换**不变**（本来就在最终系里）。
        for (auto& kv : m)
            if (kv.second.owner == r.absorbed)
                kv.second.owner = r.survivor;
        ensure(r.absorbed).owner = r.survivor;
    }

    int n = 0;
    auto put = [&](unsigned long id, const Xform& x) -> bool {
        if (n * 9 + 9 > cap)
            return false;
        float* o = out + 9 * n;
        o[0] = static_cast<float>(id);
        o[1] = x.s;
        o[2] = x.q[0]; o[3] = x.q[1]; o[4] = x.q[2]; o[5] = x.q[3];
        o[6] = x.t[0]; o[7] = x.t[1]; o[8] = x.t[2];
        ++n;
        return true;
    };

    put(display, m[display]);
    // 只给 display 那一族：链里可能还留着别的图（地图被重置后换过 current map）的合并，
    // 那些图的坐标系和 display 无关，画上去是错的。
    for (const auto& kv : m)
        if (kv.first != display && kv.second.owner == display && !put(kv.first, kv.second))
            break;
    return n;
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map* pMap_i : mspMaps)
    {
        num += pMap_i->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map* pMap_i : mspMaps) {
        num += pMap_i->GetAllMapPoints().size();
    }

    return num;
}

map<long unsigned int, KeyFrame*> Atlas::GetAtlasKeyframes()
{
    map<long unsigned int, KeyFrame*> mpIdKFs;
    for(Map* pMap_i : mvpBackupMaps)
    {
        vector<KeyFrame*> vpKFs_Mi = pMap_i->GetAllKeyFrames();

        for(KeyFrame* pKF_j_Mi : vpKFs_Mi)
        {
            mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
        }
    }

    return mpIdKFs;
}

} //namespace ORB_SLAM3
