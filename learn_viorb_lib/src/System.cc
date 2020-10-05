/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <time.h>
#include "../src/IMU/configparam.h"
#include "utils.h"
#include <boost/filesystem.hpp>
#include <unistd.h>
#include "Failure.h"
bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

bool reMapUse = false;
bool tmpbool = false;

namespace ORB_SLAM2
{
#ifdef FUNC_MAP_SAVE_LOAD
System::GenericBuilder::GenericBuilder(const std::string &strVocFile,const std::string &strSettingsFile,
                                       ORB_SLAM2::System::eSensor sensor, bool UseViewer, bool is_save_map_) :
                                       mSensor(sensor)
#else
System::GenericBuilder::GenericBuilder(const std::string &strVocFile,const std::string &strSettingsFile,
                                       ORB_SLAM2::System::eSensor sensor, bool UseViewer) :
                                       mSensor(sensor)
#endif
{

    boost::filesystem::path pathVocFileBin(strVocFile);
    pathVocFileBin.replace_extension("dbow3");
    cout << "Reading binary vocabulary from " << pathVocFileBin.c_str() << " ..." << endl;

    bool bVocLoad = false; // chose loading method based on file extension
    
    if (has_suffix(pathVocFileBin.c_str(), ".txt")){
        bVocLoad = true;
        mVocabulary.load(pathVocFileBin.c_str());
    }
    else {
        bVocLoad = true;
        mVocabulary.load(pathVocFileBin.c_str());
    }
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << pathVocFileBin.c_str() << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    
    VocFile=strVocFile;
    SettingsFile=strSettingsFile;
    bUseViewer=UseViewer;
    mpKeyFrameDatabase = std::make_unique<KeyFrameDatabase>(mVocabulary);
    mpMap=std::make_unique<Map>();
}
System::GenericBuilder::~GenericBuilder() { }
System::eSensor System::GenericBuilder::GetSensorType() { return mSensor; }
ORBVocabulary*  System::GenericBuilder::GetVocabulary() { return &mVocabulary; }
KeyFrameDatabase* System::GenericBuilder::GetKeyFrameDatabase() { return mpKeyFrameDatabase.get(); }
Map* System::GenericBuilder::GetMap() {return mpMap.get();}

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

bool System::bLocalMapAcceptKF()
{
    return (mpLocalMapper->AcceptKeyFrames() && !mpLocalMapper->isStopped());
}


void System::SaveKeyFrameTrajectoryNavState(const string &filename)
{
    cout << endl << "Saving keyframe NavState to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        Eigen::Vector3d P = pKF->GetNavState().Get_P();
        Eigen::Vector3d V = pKF->GetNavState().Get_V();
        Eigen::Quaterniond q = pKF->GetNavState().Get_R().unit_quaternion();
        Eigen::Vector3d bg = pKF->GetNavState().Get_BiasGyr();
        Eigen::Vector3d ba = pKF->GetNavState().Get_BiasAcc();
        Eigen::Vector3d dbg = pKF->GetNavState().Get_dBias_Gyr();
        Eigen::Vector3d dba = pKF->GetNavState().Get_dBias_Acc();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " ";
        f << P(0) << " " << P(1) << " " << P(2) << " ";
        f << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
        f << V(0) << " " << V(1) << " " << V(2) << " ";
        f << bg(0)+dbg(0) << " " << bg(1)+dbg(1) << " " << bg(2)+dbg(2) << " ";
        f << ba(0)+dba(0) << " " << ba(1)+dba(1) << " " << ba(2)+dba(2) << " ";
        f << endl;
    }

    f.close();
    cout << endl << "NavState trajectory saved!" << endl;
}

cv::Mat System::TrackMonoVI(const cv::Mat &im, const std::vector<IMUData> &vimu, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
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
    }
    }

    return mpTracker->GrabImageMonoVI(im,vimu,timestamp);
}

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

#ifdef FUNC_MAP_SAVE_LOAD
System::System(std::unique_ptr<GenericBuilder> builder):mpbuilder(std::move(builder)),
                mSensor(mpbuilder->GetSensorType()),
                mbReset(false),mbActivateLocalizationMode(false),
                mbDeactivateLocalizationMode(false), is_save_map(is_save_map_)
#else
System::System(std::unique_ptr<GenericBuilder> builder):mpbuilder(std::move(builder)),
                mSensor(mpbuilder->GetSensorType()),
                mbReset(false),mbActivateLocalizationMode(false),
                mbDeactivateLocalizationMode(false)
#endif
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR){
        cout << "Monocular" << endl;
    }
    else if(mSensor==STEREO){
        cout << "Stereo" << endl;
    }
    else if(mSensor==RGBD){
        cout << "RGB-D" << endl;
    }

    //Check settings file
    cv::FileStorage fsSettings(mpbuilder->SettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << mpbuilder->SettingsFile << endl;
       exit(-1);
    }

#ifdef FUNC_MAP_SAVE_LOAD
    cv::FileNode mapfilen = fsSettings["Map.mapfile"];
    bool bReuseMap = false;
    if (!mapfilen.empty())
    {
        mapfile = (string)mapfilen;
	cout << "wrote a map file" <<  endl;
    }
#endif

    mpVocabulary = mpbuilder->GetVocabulary();
    ConfigParam config(mpbuilder->SettingsFile,mSensor);

    //Create KeyFrame Database
    //Create the Map
#ifdef FUNC_MAP_SAVE_LOAD

    if (!mapfile.empty()  && LoadMap(mapfile) )
    {          
        bReuseMap = true;
    }
    
    else
#endif
    {
    //Create KeyFrame Database
    mpKeyFrameDatabase = mpbuilder->GetKeyFrameDatabase();

    //Create the Map
    mpMap = mpbuilder->GetMap();
    }
    //Create Drawers. These are used by the Viewer
#ifdef FUNC_MAP_SAVE_LOAD
    mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);
#else
    mpFrameDrawer = new FrameDrawer(mpMap);
#endif
    //mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
    mpMapDrawer = new MapDrawer(mpMap, mpbuilder->SettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)

    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
#ifdef FUNC_MAP_SAVE_LOAD
                             mpMap, mpKeyFrameDatabase, mpbuilder->SettingsFile,
                             mSensor,&config,mpbuilder->GetSubscriber(),mpbuilder->GetMPublisher(),bReuseMap);
#else
			     mpMap, mpKeyFrameDatabase, mpbuilder->SettingsFile,
                             mSensor,&config,mpbuilder->GetSubscriber(),mpbuilder->GetMPublisher());
#endif
    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR,&config);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, /*mSensor!=MONOCULAR*/true,&config);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
#ifdef FUNC_MAP_SAVE_LOAD
    mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,mpbuilder->SettingsFile, bReuseMap);
#else
    mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,mpbuilder->SettingsFile);
#endif
    if(mpbuilder->bUseViewer)
        mptViewer = new thread(&Viewer::Run, mpViewer);

    mpPublisher = mpbuilder->GetPublisher();
    mptPublisher = new thread(&ORB_SLAM2::IPublisherThread::Run, mpPublisher);

    //Initialize the publisher thread and launch

    mpTracker->SetViewer(mpViewer);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    if(ConfigParam::GetRealTimeFlag()  && !LoadMap(mapfile))
    {
      //Thread for VINS initialization
      mptLocalMappingVIOInit = new thread(&ORB_SLAM2::LocalMapping::VINSInitThread,mpLocalMapper);
    }
   
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const std::vector<IMUData> &vimu, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
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
    }
    }
    return mpTracker->GrabImageStereo(imLeft,imRight,vimu,timestamp);
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const std::vector<IMUData> &vimu, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
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
    }
    }

    return mpTracker->GrabImageRGBD(im,depthmap,vimu,timestamp);
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
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
    }
    }

    return mpTracker->GrabImageMonocular(im,timestamp);
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

void System::Remove_map()
{
    printf("Clicked remove map\n");
    unique_lock<mutex> lock(mMutexReset); 
    mpViewer->RequestStop();
    while(!mpViewer->isStopped())
        usleep(3000);
    mpMap->clear();
    mpViewer->Release(); 
    reMapUse = true;
    
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpViewer->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()  ||
          !mpViewer->isFinished()      || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    pangolin::BindToContext("ORB-SLAM2: Map Viewer");

#ifdef FUNC_MAP_SAVE_LOAD
    if (is_save_map)
        SaveMap(mapfile);
#endif
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
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
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
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
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

bool System::GetVINSInited() {
    return mpLocalMapper->GetVINSInited();
}


#ifdef FUNC_MAP_SAVE_LOAD
void System::SaveMap(const string &filename)
{
    std::ofstream out(filename, std::ios_base::binary);
    if (!out)
    {
        cerr << "Cannot Write to Mapfile: " << mapfile << std::endl;
        exit(-1);
    }
    cout << "Saving Mapfile: " << mapfile << std::flush;

    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
   
    oa << mpMap;
    oa << mpKeyFrameDatabase; 
    cout << " ...done" << std::endl;
    out.close();
}

bool System::LoadMap(const string &filename)
{
    std::ifstream in(filename, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << mapfile << " , Create a new one" << std::endl;
        return false;
    }
    cout << "Loading Mapfile: " << mapfile << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;

    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary); 
 
    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    for (auto it:vpKFS) {
        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();
        it->SetInitialNavStateAndBias(it->GetNavState());
        it->SetNavState(it->GetNavState());       
    }
    
    cout << " ...done" << endl;
    in.close();
    return true;
}

#endif
} //namespace ORB_SLAM
