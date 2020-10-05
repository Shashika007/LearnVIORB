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
#include "time.h"
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
    ros::init(argc, argv, "RGBD");
    ros::start();


    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }
    const double freq=5.0;
    ros::NodeHandle nodeHandler {"ORB_SLAM2"};
#ifdef FUNC_MAP_SAVE_LOAD  
    ORB_SLAM2::System SLAM(
            std::make_unique<ROS_System>(
                    argv[1], argv[2], ORB_SLAM2::System::RGBD, true, freq, nodeHandler));
#else
    ORB_SLAM2::System SLAM(
            std::make_unique<ROS_System>(
                    argv[1], argv[2], ORB_SLAM2::System::RGBD, freq, nodeHandler));
#endif	

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ORB_SLAM2::ConfigParam config(argv[2],2);

    /**
    * @brief added data sync
    */
    double imageMsgDelaySec = config.GetImageDelayToIMU();
    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    ros::NodeHandle nh;
    //ros::Subscriber imagesubLeft;
    //ros::Subscriber imagesubRight;
    ros::Subscriber imusub;
    if(ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
       /** message_filters::Subscriber<sensor_msgs::Image> left_sub(nh,config._imageTopic0,1);
        message_filters::Subscriber<sensor_msgs::Image> right_sub(nh,config._imageTopic1,1);
        typedef  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> sync_pol;
        message_filters::Synchronizer<sync_pol> sync(sync_pol(10),left_sub,right_sub);
        sync.registerCallback(boost::bind(&ORBVIO::MsgSynchronizer::imageCallbackStereo, &msgsync,_1,_2));
        cout<<"whats wrong"<<endl;**/

        typedef  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> sync_pol;
        message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;
        message_filters::Subscriber<sensor_msgs::Image>* depth_sub;
        message_filters::Synchronizer<sync_pol>* sync;
        rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,config._imageRGBTopic,1);
        depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,config._depthTopic,1);
        sync = new message_filters::Synchronizer<sync_pol>(sync_pol(10),*rgb_sub,*depth_sub);
        sync->registerCallback(boost::bind(&ORBVIO::MsgSynchronizer::imageCallbackStereo,&msgsync,_1,_2));

        //imagesub = nh.subscribe(config._imageTopic, /*200*/ 2, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
        imusub = nh.subscribe(config._imuTopic, 200, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);
    }
    sensor_msgs::ImageConstPtr imageMsgRGB;
    sensor_msgs::ImageConstPtr imageMsgD;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();

    ros::Rate r(1000);

    ROS_WARN("Run realtime");
    while(ros::ok())
    {
        bool bdata = msgsync.getRecentMsgsStereo(imageMsgRGB,imageMsgD,vimuMsg);


        if(bdata)
        {
            //cv::imshow("left", cv_bridge::toCvShare(imageMsgLeft)->image.clone());
            //cv::imshow("right", cv_bridge::toCvShare(imageMsgRight)->image.clone());
            //cv::waitKey();
            std::vector<ORB_SLAM2::IMUData> vimuData;
            //ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
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
                //ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
            }

            // Copy the ros image message to cv::Mat.
            cv_bridge::CvImageConstPtr cv_ptrRGB;
            cv_bridge::CvImageConstPtr cv_ptrD;
            try
            {
                cv_ptrRGB = cv_bridge::toCvShare(imageMsgRGB);
                cv_ptrD = cv_bridge::toCvShare(imageMsgD);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return -1;
            }


            cv::Mat imRGB = cv_ptrRGB->image.clone();
            cv::Mat imD = cv_ptrD->image.clone();

            {
                // To test relocalization
                static double startT=-1;
                if(startT<0)
                    startT = imageMsgRGB->header.stamp.toSec();
                // Below to test relocalizaiton
                //if(imageMsg->header.stamp.toSec() > startT+25 && imageMsg->header.stamp.toSec() < startT+25.3)
                if(imageMsgRGB->header.stamp.toSec() < startT+config._testDiscardTime){
                    imRGB = cv::Mat::zeros(imRGB.rows,imRGB.cols,imRGB.type());
                    imD = cv::Mat::zeros(imD.rows,imD.cols,imD.type());
                }
            }

            SLAM.TrackRGBD(imRGB,imD,vimuData,imageMsgRGB->header.stamp.toSec()- imageMsgDelaySec);

             //   SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                //SLAM.TrackMonoVI(cv_ptr->image, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                //cv::imshow("image",cv_ptr->image);

        }

        //cv::waitKey(1);

        ros::spinOnce();
        r.sleep();
        if(!ros::ok())
            break;
    }


//    ImageGrabber igb(&SLAM);

//    ros::NodeHandle nodeHandler;
//    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

//    ros::spin();


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

//void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
//{
//    // Copy the ros image message to cv::Mat.
//    cv_bridge::CvImageConstPtr cv_ptr;
//    try
//    {
//        cv_ptr = cv_bridge::toCvShare(msg);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

//    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
//}


