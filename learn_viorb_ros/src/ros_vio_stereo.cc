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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../learn_viorb_lib/include/System.h"

#include "MsgSync/MsgSynchronizer.h"

#include "../../learn_viorb_lib/src/IMU/imudata.h"
#include "../../learn_viorb_lib/src/IMU/configparam.h"
#include <rosbag/bag.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rosbag/view.h>
#include "utils.h"
#include "System.h"
#include "ROSPublisher.h"
#include <boost/foreach.hpp>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    //int flag=0;

    cv::Mat M1l,M2l,M1r,M2r;
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }
    const double freq=5.0;
    ros::NodeHandle nodeHandler {"ORB_SLAM2"};
#ifdef FUNC_MAP_SAVE_LOAD 
    ORB_SLAM2::System SLAM(
            std::make_unique<ROS_System>(
                    argv[1], argv[2], ORB_SLAM2::System::STEREO, true, freq, nodeHandler));
#else
    ORB_SLAM2::System SLAM(
            std::make_unique<ROS_System>(
                    argv[1], argv[2], ORB_SLAM2::System::STEREO, freq, nodeHandler));
#endif

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::ConfigParam config(argv[2],1);

    stringstream ss(argv[3]);
    bool do_rectify;
    ss >> boolalpha >> do_rectify;

    if(do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
           rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    /**
    * @brief added data sync
    */
    double imageMsgDelaySec = config.GetImageDelayToIMU();
    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    ros::NodeHandle nh;
    ros::Subscriber imusub;

    if(ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {

        typedef  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> sync_pol;
        message_filters::Subscriber<sensor_msgs::Image>* left_sub;
        message_filters::Subscriber<sensor_msgs::Image>* right_sub;
        message_filters::Synchronizer<sync_pol>* sync;
        left_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,config._imageTopic0,1);
        right_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,config._imageTopic1,1);
        sync = new message_filters::Synchronizer<sync_pol>(sync_pol(10),*left_sub,*right_sub);
        sync->registerCallback(boost::bind(&ORBVIO::MsgSynchronizer::imageCallbackStereo,&msgsync,_1,_2));

        imusub = nh.subscribe(config._imuTopic, 200, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);
    }
    sensor_msgs::ImageConstPtr imageMsgLeft;
    sensor_msgs::ImageConstPtr imageMsgRight;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();

    ros::Rate r(1000);

    ROS_WARN("Run realtime");
    while(ros::ok())
    {
        bool bdata = msgsync.getRecentMsgsStereo(imageMsgLeft,imageMsgRight,vimuMsg);
        if(bdata)
        {
            std::vector<ORB_SLAM2::IMUData> vimuData;
            for(unsigned int i=0;i<vimuMsg.size();i++)
            {
                sensor_msgs::ImuConstPtr imuMsg = vimuMsg[i];
                double ax = imuMsg->linear_acceleration.x;
                double ay = imuMsg->linear_acceleration.y;
                double az = imuMsg->linear_acceleration.z;
                if(bAccMultiply98)
                {
                    ax *= g3dm;
                    ay *= g3dm;
                    az *= g3dm;
                }
                ORB_SLAM2::IMUData imudata(imuMsg->angular_velocity.x,imuMsg->angular_velocity.y,imuMsg->angular_velocity.z,
                                ax,ay,az,imuMsg->header.stamp.toSec());
                vimuData.push_back(imudata);
            }
            // Copy the ros image message to cv::Mat.
            cv_bridge::CvImageConstPtr cv_ptrLeft;
            cv_bridge::CvImageConstPtr cv_ptrRight;
            try
            {
                cv_ptrLeft = cv_bridge::toCvShare(imageMsgLeft);
                cv_ptrRight = cv_bridge::toCvShare(imageMsgRight);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return -1;
            }
            // Consider delay of image message
            cv::Mat imLeft = cv_ptrLeft->image.clone();
            cv::Mat imRight = cv_ptrRight->image.clone();
            {
                // To test relocalization
                static double startT=-1;
                if(startT<0)
                    startT = imageMsgLeft->header.stamp.toSec();
                // Below to test relocalizaiton
                if(imageMsgLeft->header.stamp.toSec() < startT+config._testDiscardTime){
                    imLeft = cv::Mat::zeros(imLeft.rows,imLeft.cols,imLeft.type());
                    imRight = cv::Mat::zeros(imRight.rows,imRight.cols,imRight.type());
                }
            }
            if(do_rectify){
                cout<<"I did rectify"<<endl;
                cv::Mat iLeft, iRight;
                cv::remap(imLeft,iLeft,M1l,M2l,cv::INTER_LINEAR);
                cv::remap(imRight,iRight,M1r,M2r,cv::INTER_LINEAR);
                SLAM.TrackStereo(iLeft,iRight,vimuData,imageMsgLeft->header.stamp.toSec()- imageMsgDelaySec);  
            }else{
                SLAM.TrackStereo(imLeft,imRight,vimuData,imageMsgLeft->header.stamp.toSec()- imageMsgDelaySec);        
            }
        }
        ros::spinOnce();
        r.sleep();
        if(!ros::ok())
            break;
    }

    // Save camera trajectory
 
    SLAM.SaveKeyFrameTrajectoryTUM(config._tmpFilePath+"KeyFrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath+"KeyFrameNavStateTrajectory.txt");

    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}




