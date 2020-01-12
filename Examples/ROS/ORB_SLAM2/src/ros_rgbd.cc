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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    //初始化ROS系统  
    ros::init(argc, argv, "RGBD");
    //ROS启动函数
    ros::start();
    //检测给定参数数量是否完整
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // 创建SLAM系统，初始化所有系统线程并准备进程帧
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    //定义捕获图像的对象
    ImageGrabber igb(&SLAM);
    //建立ROS节点
    ros::NodeHandle nh;
    //Subscriber滤波器作为ROS消息的最顶层，不能够将其他滤波器的输出作为其输入
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/kinect2/hd/image_color_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/kinect2/hd/image_depth_rect", 1);
    //The message_filters::sync_policies::ApproximateTime policy uses an adaptive algorithm to match messages based on their timestamp.
    //下边两条指令是为了将rgb_sub和depth_sub进行数据对齐，滤掉不对齐的帧数
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    //采用registerCallback()方法，注册回调函数ImageGrabber::GrabRGBD。
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    //此函数用于ROS不断回调节点mageGrabber::GrabRGBD函数
    ros::spin();

    // Stop all threads 停止所有线程
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

/*******************************************************************************
 *     函数属性：类ImageGrabber的成员函数
 *     函数功能：（1）将ROS的图片信息转变为cv::Mat的形式
 *                        （2）将获取的图片信息加入SLAM系统的成员函数TrackRGBD()中进行处理
 *     函数参数介绍：
 *                          msgRGB：通过Kinect获得的RGB彩色图像（ROS图像类型）
 *                          msgD：     通过Kinect获得的深度图像     （ROS图像类型）
 *     备注：此函数为节点的回调函数，通过不断回调该函数，来实现SLAM系统的同时定位与地图构建功能
 * 
 ******************************************************************************/

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}


