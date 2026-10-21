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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <iomanip>
#include <stdexcept>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include "md5.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

namespace ORB_SLAM3
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const int initFr, const string &strSequence):
    mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbResetActiveMap(false),
    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDown(false),
    mnWaypointRevision(0), mnNextWaypointId(1)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;
    else if(mSensor==IMU_STEREO)
        cout << "Stereo-Inertial" << endl;
    else if(mSensor==IMU_RGBD)
        cout << "RGB-D-Inertial" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if(!node.empty() && node.isString() && node.string() == "1.0"){
        settings_ = new Settings(strSettingsFile,mSensor);

        mStrLoadAtlasFromFile = settings_->atlasLoadFile();
        mStrSaveAtlasToFile = settings_->atlasSaveFile();

        cout << (*settings_) << endl;
    }
    else{
        settings_ = nullptr;
        cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
        if(!node.empty() && node.isString())
        {
            mStrLoadAtlasFromFile = (string)node;
        }

        node = fsSettings["System.SaveAtlasToFile"];
        if(!node.empty() && node.isString())
        {
            mStrSaveAtlasToFile = (string)node;
        }
    }

    node = fsSettings["loopClosing"];
    bool activeLC = true;
    if(!node.empty())
    {
        activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
    }

    mStrVocabularyFilePath = strVocFile;

    bool loadedAtlas = false;

    if(mStrLoadAtlasFromFile.empty())
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create the Atlas
        cout << "Initialization of Atlas from scratch " << endl;
        mpAtlas = new Atlas(0);
    }
    else
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        cout << "Load File" << endl;

        // Load the file with an earlier session
        //clock_t start = clock();
        cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        if(!isRead)
        {
            cout << "Error to load the file, please try with other session file or vocabulary file" << endl;
            // 上游是 exit(-1)：在安卓上就是 App 直接没了、不给任何提示。抛出去让调用方接。
            throw std::runtime_error("LoadAtlas failed: " + mStrLoadAtlasFromFile + ".osa");
        }
        //mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);


        //cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " << mpKeyFrameDatabase->mnNumWords << endl;

        loadedAtlas = true;

        mpAtlas->CreateNewMap();

        //clock_t timeElapsed = clock() - start;
        //unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
        //cout << "Binary file read in " << msElapsed << " ms" << endl;

        //usleep(10*1000*1000);
    }


    if (mSensor==IMU_STEREO || mSensor==IMU_MONOCULAR || mSensor==IMU_RGBD)
        mpAtlas->SetInertialSensor();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    cout << "Seq. Name: " << strSequence << endl;
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_, strSequence);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR,
                                     mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD, strSequence);
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper);
    mpLocalMapper->mInitFr = initFr;
    if(settings_)
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    else
        mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    if(mpLocalMapper->mThFarPoints!=0)
    {
        cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
        mpLocalMapper->mbFarPoints = false;

    //Initialize the Loop Closing thread and launch
    // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR, activeLC); // mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    //usleep(10*1000*1000);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    //if(false) // TODO
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
    }

    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);

}

Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=STEREO && mSensor!=IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if(settings_ && settings_->needToRectify()){
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if(settings_ && settings_->needToResize()){
        cv::resize(imLeft,imLeftToFeed,settings_->newImSize());
        cv::resize(imRight,imRightToFeed,settings_->newImSize());
    }
    else{
        imLeftToFeed = imLeft.clone();
        imRightToFeed = imRight.clone();
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    // std::cout << "start GrabImageStereo" << std::endl;
    Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed,imRightToFeed,timestamp,filename);

    // std::cout << "out grabber" << std::endl;

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

Sophus::SE3f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=RGBD  && mSensor!=IMU_RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    cv::Mat imDepthToFeed = depthmap.clone();
    if(settings_ && settings_->needToResize()){
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;

        cv::resize(depthmap,imDepthToFeed,settings_->newImSize());
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_RGBD)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed,imDepthToFeed,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{

    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbShutDown)
            return Sophus::SE3f();
    }

    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    if(settings_ && settings_->needToResize()){
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            cout << "SYSTEM-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}



void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::Shutdown()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbShutDown = true;
    }

    cout << "Shutdown" << endl;

    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    /*if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }*/

    // Wait until all thread have effectively stopped
    /*while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        if(!mpLocalMapper->isFinished())
            cout << "mpLocalMapper is not finished" << endl;*/
        /*if(!mpLoopCloser->isFinished())
            cout << "mpLoopCloser is not finished" << endl;
        if(mpLoopCloser->isRunningGBA()){
            cout << "mpLoopCloser is running GBA" << endl;
            cout << "break anyway..." << endl;
            break;
        }*/
        /*usleep(5000);
    }*/

    if(!mStrSaveAtlasToFile.empty())
    {
        Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
        SaveAtlas(FileType::BINARY_FILE);
    }

    /*if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");*/

#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif


}

bool System::isShutDown() {
    unique_lock<mutex> lock(mMutexReset);
    return mbShutDown;
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }
    f.close();
    // cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
          << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

    }

    f.close();
}

void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    int numMaxKFs = 0;
    Map* pBiggerMap;
    std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
    for(Map* pMap :vpMaps)
    {
        std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Twc = ((*lit)*Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

int System::GetFrameTrajectorySize()
{
    if (mpTracker == nullptr)
        return 0;
    unique_lock<mutex> lockTraj(mpTracker->mMutexTraj);
    return static_cast<int>(mpTracker->mlRelativeFramePoses.size());
}

long long System::GetTrajectoryMapId()
{
    if (mpTracker == nullptr)
        return -1;

    // 判据和 `GetFrameTrajectory()` 逐字一致 —— 那边也是这么挑 `pMap` 的。
    unique_lock<mutex> lockTraj(mpTracker->mMutexTraj);
    Map* pMap = nullptr;
    for (list<KeyFrame*>::const_reverse_iterator it = mpTracker->mlpReferences.rbegin();
         it != mpTracker->mlpReferences.rend(); ++it)
        if (*it) { pMap = (*it)->GetMap(); break; }

    return pMap == nullptr ? -1 : static_cast<long long>(pMap->GetId());
}

int System::GetMergeRecords(float* out, int cap)
{
    if (mpAtlas == nullptr)
        return 0;
    return mpAtlas->GetMergeRecords(out, cap);
}

void System::ClearFrameTrajectory()
{
    if (mpTracker != nullptr)
        mpTracker->ClearTrajectory();
}

int System::GetFrameTrajectory(float* out, int cap)
{
    if (out == nullptr || mpTracker == nullptr)
        return 0;

    // ⚠️ 必须拿这把锁：`Tracking` 线程正在往这四张表里 push，而 `System::mMutexState`
    //    不包括它们（那个只保护三个快照字段）。
    unique_lock<mutex> lockTraj(mpTracker->mMutexTraj);

    const list<Sophus::SE3f>& vRel = mpTracker->mlRelativeFramePoses;
    const list<KeyFrame*>& vRef = mpTracker->mlpReferences;
    const list<double>& vT = mpTracker->mlFrameTimes;
    const list<bool>& vLost = mpTracker->mlbLost;

    // 四张表平行；短的那个当上界。
    size_t n = vRel.size();
    n = std::min(n, vRef.size());
    n = std::min(n, vT.size());
    n = std::min(n, vLost.size());
    if (n == 0 || cap < 8)
        return 0;

    const size_t maxPts = std::min<size_t>(cap / 8, 4096);
    const size_t stride = std::max<size_t>(1, (n + maxPts - 1) / maxPts);

    // ⚠️ 不用 `mpAtlas->GetCurrentMap()` —— 它在坏图窗口里是 `while(IsBad()) usleep(3000)`，
    //    从主线程调会**冻死界面**。改用「最后一个有效参考帧属于哪张图」，语义一样、不会挂。
    //    附带好处：重置之后老帧的参考帧在旧图上 ⟹ **自动被滤掉**，不用自己清轨迹。
    Map* pMap = nullptr;
    for (list<KeyFrame*>::const_reverse_iterator it = vRef.rbegin(); it != vRef.rend(); ++it)
        if (*it) { pMap = (*it)->GetMap(); break; }
    if (pMap == nullptr)
        return 0;

    // 一帧的拼装。`rel` 是「相对 `pKF`」的位姿（当场记的，**不变**），
    // `pKF->GetPose()` 是**会变**的（BA 一直在优化它）—— 现乘 ⟹ 轨迹自动跟着优化走。
    // ⚠️ 和上游 `SaveTrajectoryEuRoC` 有两处**故意不同**：
    //    · **不乘 `pKF->mImuCalib.mTbc`** —— 那会转到 body/IMU 系，而实时位姿是**相机系**；
    //    · **不乘首帧偏移 `Twb`** —— 那会挪到 EuRoC 的真值系，而我们要的是**地图坐标系**。
    //    两条都是为了「轨迹和当前位置（`poseBuf`）在同一个系里」，否则两者会错开。
    auto compose = [&](const Sophus::SE3f& rel, KeyFrame* pKF, double t, float* o) -> bool {
        if (pKF == nullptr) return false;
        // 参考帧被剔了就沿生成树往上爬（上游原样）。
        // ⚠️ 上游这个循环**没有上界** —— 链坏了（成环 / 爬到 null）就转不出来。加一个。
        Sophus::SE3f Trw;
        int hops = 0;
        while (pKF->isBad())
        {
            if (++hops > 64) return false;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            if (pKF == nullptr) return false;
        }
        if (pKF->GetMap() != pMap) return false;   // 别的图的帧（重置之前那些）

        const Sophus::SE3f Twc = (rel * Trw * pKF->GetPose()).inverse();
        const Eigen::Quaternionf q = Twc.unit_quaternion();
        const Eigen::Vector3f tw = Twc.translation();
        o[0] = static_cast<float>(t);
        o[1] = tw(0); o[2] = tw(1); o[3] = tw(2);
        o[4] = q.x(); o[5] = q.y(); o[6] = q.z(); o[7] = q.w();
        return true;
    };

    // ⚠️ 四个迭代器**一起推** —— 关联的是「位置」，不是「算过没算」。
    //    所以跳过一万条不拼装，「rel[i] 配 refs[i]」也永远不破。
    list<Sophus::SE3f>::const_iterator itRel = vRel.begin();
    list<KeyFrame*>::const_iterator itRef = vRef.begin();
    list<double>::const_iterator itT = vT.begin();
    list<bool>::const_iterator itLost = vLost.begin();

    int m = 0;                     // 已写出的点数
    size_t emitted = 0;            // 走过的**有效**帧数（跟丢的不算）
    Sophus::SE3f lastRel;          // 最后一个有效帧的那几个值 —— 存下来很便宜
    KeyFrame* lastKF = nullptr;
    double lastT = 0;
    bool hasLast = false;

    for (size_t i = 0; i < n; ++i, ++itRel, ++itRef, ++itT, ++itLost)
    {
        if (*itLost) continue;     // 跟丢的帧不存（上游也这么干）
        lastRel = *itRel; lastKF = *itRef; lastT = *itT; hasLast = true;

        // ⚠️ 按**有效帧**计数，不按 list 下标 —— 按下标的话，跟丢一帧后面整条网格就错位了。
        // ⚠️ `continue` **在 `compose` 之前** —— 这才是抽稀的意义：
        //    走列表只是推指针（便宜），矩阵乘只做选中的那 ≤maxPts 个。
        const size_t slot = emitted++;
        if (slot % stride != 0) continue;
        if (m * 8 + 8 > cap) break;
        if (compose(lastRel, lastKF, lastT, out + 8 * m)) ++m;
    }

    // 补最后一个**有效帧**，让轨迹的头贴着当前位置
    //（它不在网格上时才补 —— 在的话上面已经写过了，再补就重复）。
    if (hasLast && m > 0 && (emitted - 1) % stride != 0 && (m + 1) * 8 <= cap)
        if (compose(lastRel, lastKF, lastT, out + 8 * m)) ++m;

    return m;
}

std::vector<MapPoint*> System::GetAllMapPoints()
{
    if (mpAtlas == nullptr)
        return std::vector<MapPoint*>();
    return mpAtlas->GetAllMapPoints();
}

// ---------------- add：路径点 ----------------

struct System::Waypoint
{
    int id = 0;
    std::string name;
    KeyFrame* pAnchor = nullptr;   // 非拥有；只在图集合里取
    Sophus::SE3f T_rel;            // `Tcw_此刻 * Twc_锚点`
};

namespace {

/**
 * 把「锚在 `pKF` 上的路径点」解到世界坐标。
 *
 * 和 `GetFrameTrajectory()` 里那个 `compose` 是**同一套算法**：
 * 锚点坏了就沿 `mTcp`（它标坏那一刻记下的「相对父帧」）往上爬，爬到活着的祖先。
 * 所以关键帧被正常剔除**不影响坐标**；只有 `GetMap() == nullptr`
 * （`Map::clear()`，即地图被重置）才算失效 —— 那时坐标系真的没了。
 *
 * @param T_rel  路径点相对锚点的位姿
 * @param pKF    锚点
 * @param Tcw    出参：路径点在世界里的 `Tcw`
 * @param mapId  出参：解析到的**活着的**祖先落在哪张图
 * @return false = 这个路径点现在没有有效坐标
 */
bool ResolveWaypoint(const Sophus::SE3f& T_rel, KeyFrame* pKF,
                     Sophus::SE3f& Tcw, unsigned long& mapId)
{
    if (pKF == nullptr)
        return false;

    Sophus::SE3f Trw;
    int hops = 0;
    while (pKF->isBad())
    {
        if (++hops > 64)        // 上游那个循环没有上界；链坏了（成环 / 爬到 null）就转不出来
            return false;
        Trw = Trw * pKF->mTcp;
        pKF = pKF->GetParent();
        if (pKF == nullptr)
            return false;
    }

    Map* pMap = pKF->GetMap();
    if (pMap == nullptr)
        return false;

    Tcw = T_rel * Trw * pKF->GetPose();
    mapId = pMap->GetId();
    return true;
}

/** `T_rel` 里那个 `\t` / `\n` / `\\` 的转义。名字是用户输入的，会带空格。 */
std::string EscapeWpName(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s)
    {
        if (c == '\\') out += "\\\\";
        else if (c == '\t') out += "\\t";
        else if (c == '\n') out += "\\n";
        else out += c;
    }
    return out;
}

bool UnescapeWpName(const std::string& s, std::string& out)
{
    out.clear();
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i)
    {
        if (s[i] != '\\') { out += s[i]; continue; }
        if (++i >= s.size()) return false;
        if (s[i] == '\\') out += '\\';
        else if (s[i] == 't') out += '\t';
        else if (s[i] == 'n') out += '\n';
        else { out += '\\'; out += s[i]; }   // 不认识的转义原样留着
    }
    return true;
}

}  // namespace

bool System::AddWaypoint(const std::string& name)
{
    if (mpAtlas == nullptr || mpTracker == nullptr)
        return false;

    // 用 `PeekCurrentMap()`：`GetCurrentMap()` 在图被标坏的窗口里会抱着 atlas 锁
    // `usleep` 自旋，而这里是 SLAM 线程，卡住等于整个 App 不跟手。
    Map* pMap = mpAtlas->PeekCurrentMap();
    if (pMap == nullptr)
    {
        std::lock_guard<mutex> lk(mMutexWaypoints);
        mstrWaypointError = "没有当前图";
        return false;
    }

    // ⚠️ 锚点必须**已经在图集合里**：还在 `LocalMapping` 队列里的 KF 会被
    //    `Release()` 直接 delete，拿它做锚点就是悬空指针。
    KeyFrame* pAnchor = pMap->GetLatestKeyFrame();
    if (pAnchor == nullptr)
    {
        std::lock_guard<mutex> lk(mMutexWaypoints);
        mstrWaypointError = "图 " + std::to_string(pMap->GetId()) + " 还没有关键帧";
        return false;
    }

    const Frame& F = mpTracker->mCurrentFrame;
    if (!F.isSet())
    {
        std::lock_guard<mutex> lk(mMutexWaypoints);
        mstrWaypointError = "当前帧没有位姿";
        return false;
    }

    Waypoint* p = new Waypoint;
    p->name = name.empty() ? "路径点" : name;
    p->pAnchor = pAnchor;
    p->T_rel = F.GetPose() * pAnchor->GetPoseInverse();

    {
        unique_lock<mutex> lock(mMutexWaypoints);
        p->id = mnNextWaypointId++;
        mvpWaypoints.push_back(p);
        ++mnWaypointRevision;
        mstrWaypointError.clear();
    }

    cout << "[WP] 加路径点 #" << p->id << " 「" << p->name << "」 → 锚 KF "
         << pAnchor->mnId << "（图 " << pMap->GetId() << "）" << endl;
    return true;
}

std::string System::GetWaypointError()
{
    std::lock_guard<mutex> lk(mMutexWaypoints);
    return mstrWaypointError;
}

void System::RenameWaypoint(int id, const std::string& name)
{
    unique_lock<mutex> lock(mMutexWaypoints);
    for (Waypoint* p : mvpWaypoints)
    {
        if (p->id != id) continue;
        p->name = name;
        ++mnWaypointRevision;
        return;
    }
}

void System::DeleteWaypoint(int id)
{
    unique_lock<mutex> lock(mMutexWaypoints);
    for (std::vector<Waypoint*>::iterator it = mvpWaypoints.begin(); it != mvpWaypoints.end(); ++it)
    {
        if ((*it)->id != id) continue;
        delete *it;
        mvpWaypoints.erase(it);
        ++mnWaypointRevision;
        return;
    }
}

void System::ClearWaypoints()
{
    unique_lock<mutex> lock(mMutexWaypoints);
    for (Waypoint* p : mvpWaypoints)
        delete p;
    mvpWaypoints.clear();
    mnNextWaypointId = 1;
    ++mnWaypointRevision;
}

int System::GetWaypointCount()
{
    unique_lock<mutex> lock(mMutexWaypoints);
    return static_cast<int>(mvpWaypoints.size());
}

int System::GetNextWaypointId()
{
    unique_lock<mutex> lock(mMutexWaypoints);
    return mnNextWaypointId;
}

std::string System::GetWaypointName(int id)
{
    unique_lock<mutex> lock(mMutexWaypoints);
    for (Waypoint* p : mvpWaypoints)
        if (p->id == id) return p->name;
    return std::string();
}

long long System::GetWaypointRevision()
{
    unique_lock<mutex> lock(mMutexWaypoints);
    return mnWaypointRevision;
}

int System::GetWaypoints(float* pos, int posCap, long long* meta, int metaCap)
{
    if (pos == nullptr || meta == nullptr)
        return 0;

    // 两条出参与原来一样「装不下就少给」：返回**实际写了几个**。
    //    调用方看到写满就扩容，下一拍再来 —— 不能返回真实条数。
    const int canPos = posCap / 3;
    const int canMeta = metaCap / 4;
    const int cap = std::min(canPos, canMeta);
    if (cap <= 0)
        return 0;

    // 用 `PeekCurrentMap()` 而不是 `GetCurrentMap()` —— 后者在坏图窗口里
    //    会 `while(IsBad()) usleep(3000)` 抱着 atlas 锁睡。这条是从主线程读的。
    Map* pCur = mpAtlas == nullptr ? nullptr : mpAtlas->PeekCurrentMap();
    const unsigned long curId = pCur == nullptr ? 0 : pCur->GetId();

    // 持 `mMutexWaypoints` 期间会去拿 KF 自己的锁（`isBad` / `GetPose` / `GetMap`）。
    //    只有一个方向（waypoints → KF），没有反向路径。
    unique_lock<mutex> lock(mMutexWaypoints);

    int m = 0;
    for (Waypoint* p : mvpWaypoints)
    {
        if (m >= cap)
            break;

        Sophus::SE3f Tcw;
        unsigned long mapId = 0;
        const bool ok = ResolveWaypoint(p->T_rel, p->pAnchor, Tcw, mapId);

        long long flags = 0;
        if (ok)
        {
            flags |= 1;                     // bit0 坐标有效
            if (mapId == curId) flags |= 2; // bit1 在当前图
            const Eigen::Vector3f t = Tcw.inverse().translation();
            pos[3 * m] = t.x();
            pos[3 * m + 1] = t.y();
            pos[3 * m + 2] = t.z();
        }
        else
        {
            pos[3 * m] = pos[3 * m + 1] = pos[3 * m + 2] = 0.f;
        }

        long long* md = meta + 4 * m;
        md[0] = p->id;
        md[1] = ok ? static_cast<long long>(mapId) : -1;
        md[2] = p->pAnchor != nullptr ? static_cast<long long>(p->pAnchor->mnId) : -1;
        md[3] = flags;
        ++m;
    }
    return m;
}

bool System::SaveWaypoints(const std::string& path)
{
    if (path.empty())
        return false;

    const std::string file = path + ".waypoints.tsv";
    const std::string tmp = file + ".tmp";

    std::ofstream f(tmp, std::ios::trunc);
    if (!f.is_open())
    {
        cout << "[WP] 写不出去：" << tmp << endl;
        return false;
    }
    f << "# slam-waypoints 1\n";

    unique_lock<mutex> lock(mMutexWaypoints);
    int saved = 0, skipped = 0;
    for (Waypoint* p : mvpWaypoints)
    {
        // 沿坏链爬到活着的祖先：存**祖先 id + 修正后的 T_rel**，读的时候不用再爬。
        KeyFrame* pKF = p->pAnchor;
        if (pKF == nullptr) { ++skipped; continue; }

        Sophus::SE3f Trw;
        int hops = 0;
        bool ok = true;
        while (pKF->isBad())
        {
            if (++hops > 64) { ok = false; break; }
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            if (pKF == nullptr) { ok = false; break; }
        }
        if (!ok || pKF->GetMap() == nullptr) { ++skipped; continue; }   // 失效的不落盘

        const Sophus::SE3f T_rel = p->T_rel * Trw;
        const Eigen::Quaternionf q = T_rel.unit_quaternion();
        const Eigen::Vector3f t = T_rel.translation();

        f << p->id << '\t' << pKF->GetMap()->GetId() << '\t' << pKF->mnId << '\t'
          << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
          << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\t'
          << EscapeWpName(p->name) << '\n';
        ++saved;
    }
    f.close();

    if (!f.good())
    {
        std::remove(tmp.c_str());
        cout << "[WP] 写 sidecar 失败" << endl;
        return false;
    }
    if (std::rename(tmp.c_str(), file.c_str()) != 0)
    {
        std::remove(tmp.c_str());
        cout << "[WP] 改名失败：" << file << endl;
        return false;
    }

    cout << "[WP] 路径点已保存 " << saved << " 条" << (skipped ? "（跳过失效 " + std::to_string(skipped) + "）" : "")
         << " → " << file << endl;
    return true;
}

bool System::LoadWaypoints(const std::string& path)
{
    ClearWaypoints();
    if (path.empty())
        return true;

    const std::string file = path + ".waypoints.tsv";
    std::ifstream f(file);
    if (!f.is_open())
        return true;   // 没有 sidecar 是正常情况（这张图从没加过路径点）

    // kfId → KeyFrame*。⚠️ 不能用 `Atlas::GetAtlasKeyframes()` —— 那个读的是
    // `mvpBackupMaps`，`PostLoad()` 之后已经清空了。
    std::map<unsigned long, KeyFrame*> byId;
    if (mpAtlas != nullptr)
    {
        for (Map* pMap : mpAtlas->GetAllMaps())
        {
            if (pMap == nullptr) continue;
            for (KeyFrame* pKF : pMap->GetAllKeyFrames())
                if (pKF != nullptr) byId[pKF->mnId] = pKF;
        }
    }

    std::string line;
    int loaded = 0, missing = 0, bad = 0;
    int maxId = 0;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
            continue;

        // id \t mapId \t kfId \t tx ty tz qx qy qz qw \t name
        const size_t t1 = line.find('\t');
        if (t1 == std::string::npos) { ++bad; continue; }
        const size_t t2 = line.find('\t', t1 + 1);
        if (t2 == std::string::npos) { ++bad; continue; }
        const size_t t3 = line.find('\t', t2 + 1);
        if (t3 == std::string::npos) { ++bad; continue; }

        std::istringstream head(line.substr(0, t3));
        int id = 0;
        unsigned long mapId = 0, kfId = 0;
        float tx = 0, ty = 0, tz = 0, qx = 0, qy = 0, qz = 0, qw = 1;
        if (!(head >> id >> mapId >> kfId >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
        {
            ++bad;
            continue;
        }

        std::string name;
        if (!UnescapeWpName(line.substr(t3 + 1), name))
            name = "路径点";

        Waypoint* p = new Waypoint;
        p->id = id;
        p->name = name;
        p->T_rel = Sophus::SE3f(Eigen::Quaternionf(qw, qx, qy, qz).normalized(), Eigen::Vector3f(tx, ty, tz));

        auto it = byId.find(kfId);
        if (it != byId.end())
        {
            p->pAnchor = it->second;
            ++loaded;
        }
        else
        {
            // 锚点不在这份 atlas 里（覆盖保存过、或那张图被剔掉了）。**保留名字、标失效**，
            // 让用户能在管理界面看到并手动删 —— 不猜、不静默改写成别的锚点。
            p->pAnchor = nullptr;
            ++missing;
        }
        if (id > maxId) maxId = id;

        std::lock_guard<mutex> lock(mMutexWaypoints);
        mvpWaypoints.push_back(p);
    }

    {
        unique_lock<mutex> lock(mMutexWaypoints);
        mnNextWaypointId = maxId + 1;
        ++mnWaypointRevision;
    }

    cout << "[WP] 读路径点 " << (loaded + missing) << " 条（有效 " << loaded
         << " / 失效 " << missing << " / 坏行 " << bad << "）：" << file << endl;
    return true;
}

void System::SaveAtlasTo(const std::string& path)
{
    // 文件名现取：塞进那个成员再走上游那条路（SaveAtlas 只认成员，不带路径参数）。
    mStrSaveAtlasToFile = path;

    // 先停 LocalMapping：它正在建点的话，序列化会读到半张图。停不动就别死等，直接存。
    if (mpLocalMapper != nullptr)
    {
        mpLocalMapper->RequestStop();
        for (int i = 0; i < 200 && !mpLocalMapper->Stop(); ++i)
            usleep(1000);
    }

    SaveAtlas(FileType::BINARY_FILE);

    if (mpLocalMapper != nullptr)
        mpLocalMapper->Release();
}

bool System::ReloadAtlasFrom(const std::string& path)
{
    if(mpTracker == nullptr || mpAtlas == nullptr)
        return false;

    // 路径点属于旧 atlas：换掉之前先清空。失败时也不能留着（坐标已经没有意义）。
    ClearWaypoints();
    // 合并链同理：后面 `LoadAtlasInto` 成功时 `PostLoad()` 也会清，这里兜住失败那条路。
    mpAtlas->ClearMergeRecords();

    // 清当前图、轨迹和倒排表；KeyFrame / Frame 的 ID 计数器归零
    //（紧接着的 LoadAtlasInto 会把它们恢复成文件里的值）。
    // ⚠️ 这一步会阻塞等 LocalMapping / LoopClosing 应答，所以只能从 SLAM 线程调。
    mpTracker->Reset();

    if(path.empty())
        return true;   // 上面那次 Reset 已经留了一张空图

    if(!LoadAtlasInto(path, FileType::BINARY_FILE))
    {
        cout << "ReloadAtlasFrom: 载入失败，保留一张空图" << endl;
        return false;
    }

    // Reset 建的那张空图不在新读进来的 mspMaps 里了 ⟹ 换一张干净的当前图。
    mpAtlas->CreateNewMap();
    mpAtlas->SetInertialSensor();
    return true;
}

void System::GetAtlasStats(int* maps, int* kfs, int* mps)
{
    *maps = 0;
    *kfs = 0;
    *mps = 0;
    if (mpAtlas == nullptr)
        return;

    // `GetAllMaps()` 拿 `mMutexAtlas`、`KeyFramesInMap()`/`MapPointsInMap()` 拿 `mMutexMap`
    // ⟹ 从任意线程调都安全（不碰 `mpCurrentMap`，所以也不会撞上 `GetCurrentMap()` 那个睡眠）。
    for (Map* pMap : mpAtlas->GetAllMaps())
    {
        if (pMap == nullptr)
            continue;
        (*maps)++;
        *kfs += static_cast<int>(pMap->KeyFramesInMap());
        *mps += static_cast<int>(pMap->MapPointsInMap());
    }
}

void System::SaveTrajectoryEuRoC(const string &filename, Map* pMap)
{

    cout << endl << "Saving trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    int numMaxKFs = 0;

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Twc = ((*lit)*Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

/*void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose_();
    else
        Twb = vpKFs[0]->GetPoseInverse_();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;


        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
            Sophus::SE3f Twb = Tbw.inverse();

            Eigen::Vector3f twb = Twb.translation();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}*/


/*void System::SaveKeyFrameTrajectoryEuRoC_old(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    }
    f.close();
}*/

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap)
{
    cout << endl << "Saving keyframe trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

/*void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPoseCv() * Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
}*/

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        if(!pKF)
            continue;

        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Tow;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();

        f << setprecision(9) << Rwc(0,0) << " " << Rwc(0,1)  << " " << Rwc(0,2) << " "  << twc(0) << " " <<
             Rwc(1,0) << " " << Rwc(1,1)  << " " << Rwc(1,2) << " "  << twc(1) << " " <<
             Rwc(2,0) << " " << Rwc(2,1)  << " " << Rwc(2,2) << " "  << twc(2) << endl;
    }
    f.close();
}


void System::SaveDebugData(const int &initIdx)
{
    // 0. Save initialization trajectory
    SaveTrajectoryEuRoC("init_FrameTrajectoy_" +to_string(mpLocalMapper->mInitSect)+ "_" + to_string(initIdx)+".txt");

    // 1. Save scale
    ofstream f;
    f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mScale << endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mRwg(0,0) << "," << mpLocalMapper->mRwg(0,1) << "," << mpLocalMapper->mRwg(0,2) << endl;
    f << mpLocalMapper->mRwg(1,0) << "," << mpLocalMapper->mRwg(1,1) << "," << mpLocalMapper->mRwg(1,2) << endl;
    f << mpLocalMapper->mRwg(2,0) << "," << mpLocalMapper->mRwg(2,1) << "," << mpLocalMapper->mRwg(2,2) << endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mCostTime << endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2) << endl;
    f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2) << endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" +to_string(mpLocalMapper->mInitSect)+ "_" +to_string(initIdx)+".txt", ios_base::app);
    f << fixed;
    for(int i=0; i<mpLocalMapper->mcovInertial.rows(); i++)
    {
        for(int j=0; j<mpLocalMapper->mcovInertial.cols(); j++)
        {
            if(j!=0)
                f << ",";
            f << setprecision(15) << mpLocalMapper->mcovInertial(i,j);
        }
        f << endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mInitTime << endl;
    f.close();
}


int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    if ((aux>0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        if ((mpTracker->mState==Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }
}


bool System::isFinished()
{
    return (GetTimeFromIMUInit()>0.1);
}

void System::ChangeDataset()
{
    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

float System::GetImageScale()
{
    return mpTracker->GetImageScale();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time)
{
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double& time)
{
    mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double& time)
{
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

void System::SaveAtlas(int type){
    if(!mStrSaveAtlasToFile.empty())
    {
        //clock_t start = clock();

        // Save the current session
        mpAtlas->PreSave();

        // 上游这里硬拼了 "./" 前缀，会把绝对路径变成相对的（安卓的 CWD 是 `/`，不可写）。
        string pathSaveFileName = mStrSaveAtlasToFile + ".osa";

        string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);
        std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
        string strVocabularyName = mStrVocabularyFilePath.substr(found+1);

        if(type == TEXT_FILE) // File text
        {
            cout << "Starting to write the save text file " << endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::text_oarchive oa(ofs);

            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            cout << "End to write the save text file" << endl;
        }
        else if(type == BINARY_FILE) // File binary
        {
            cout << "Starting to write the save binary file" << endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            cout << "End to write save binary file" << endl;
        }
    }
}

bool System::LoadAtlas(int type)
{
    string strFileVoc, strVocChecksum;
    bool isRead = false;

    // 同 SaveAtlas：那个 "./" 前缀会把绝对路径变成相对的。
    string pathLoadFileName = mStrLoadAtlasFromFile + ".osa";

    if(type == TEXT_FILE) // File text
    {
        cout << "Starting to read the save text file " << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if(!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::text_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        cout << "End to load the save text file " << endl;
        isRead = true;
    }
    else if(type == BINARY_FILE) // File binary
    {
        cout << "Starting to read the save binary file"  << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if(!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::binary_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        cout << "End to load the save binary file" << endl;
        isRead = true;
    }

    if(isRead)
    {
        //Check if the vocabulary is the same
        string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);

        if(strInputVocabularyChecksum.compare(strVocChecksum) != 0)
        {
            cout << "The vocabulary load isn't the same which the load session was created " << endl;
            cout << "-Vocabulary name: " << strFileVoc << endl;
            return false; // Both are differents
        }

        mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
        mpAtlas->SetORBVocabulary(mpVocabulary);
        mpAtlas->PostLoad();

        return true;
    }
    return false;
}

bool System::LoadAtlasInto(const std::string& path, int type)
{
    string strFileVoc, strVocChecksum;
    bool isRead = false;
    Atlas* pLoaded = nullptr;

    string pathLoadFileName = path + ".osa";

    try
    {
        if(type == TEXT_FILE)
        {
            cout << "Starting to read the save text file " << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if(!ifs.good())
            {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::text_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> pLoaded;
            isRead = true;
        }
        else if(type == BINARY_FILE)
        {
            cout << "Starting to read the save binary file" << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if(!ifs.good())
            {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::binary_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> pLoaded;
            isRead = true;
        }
    }
    catch(const std::exception& e)
    {
        cout << "LoadAtlasInto: 反序列化失败 " << e.what() << endl;
        delete pLoaded;
        return false;
    }

    if(!isRead || pLoaded == nullptr)
    {
        delete pLoaded;
        return false;
    }

    //Check if the vocabulary is the same
    string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
    if(strInputVocabularyChecksum.compare(strVocChecksum) != 0)
    {
        cout << "The vocabulary load isn't the same which the load session was created " << endl;
        cout << "-Vocabulary name: " << strFileVoc << endl;
        delete pLoaded;
        return false;
    }

    // 关键差别：不 `mpAtlas = pLoaded`（那会让各线程手里的指针失效），
    // 而是把内容搬进现有的 Atlas。
    mpAtlas->AdoptFrom(pLoaded);
    delete pLoaded;   // 内容已搬走，只剩空壳（`mspMaps` 还是空的）

    mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    mpAtlas->SetORBVocabulary(mpVocabulary);
    mpAtlas->PostLoad();

    return true;
}

string System::CalculateCheckSum(string filename, int type)
{
    string checksum = "";

    unsigned char c[MD5_DIGEST_LENGTH];

    std::ios_base::openmode flags = std::ios::in;
    if(type == BINARY_FILE) // Binary file
        flags = std::ios::in | std::ios::binary;

    ifstream f(filename.c_str(), flags);
    if ( !f.is_open() )
    {
        cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
        return checksum;
    }

    MD5_CTX md5Context;
    char buffer[1024];

    MD5_Init (&md5Context);
    while ( int count = f.readsome(buffer, sizeof(buffer)))
    {
        MD5_Update(&md5Context, buffer, count);
    }

    f.close();

    MD5_Final(c, &md5Context );

    for(int i = 0; i < MD5_DIGEST_LENGTH; i++)
    {
        char aux[10];
        sprintf(aux,"%02x", c[i]);
        checksum = checksum + aux;
    }

    return checksum;
}

} //namespace ORB_SLAM

