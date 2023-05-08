/* ----------------------------------------------------------------------------
 * Copyright 2023, Speike <shao-haoluo@foxmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file       feature_tracker_node.cpp
 * @author     Speike
 * @date       2023/04/14 16:51:59
 * @brief      特征提取节点，参考VINS-MONO的对应文件
**/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include "feature_tracker.h"
#include <boost/filesystem.hpp>

ros::Publisher restart_pub;         //发布feature_tracker的状态->是否重启
ros::Publisher feature_points_pub;  //发布跟踪的特征点,2d点云形式
ros::Publisher feature_img_pub;     //发布带跟踪特征点的图像
FeatureTracker trackerData;         //特征跟踪类对象

double first_image_flag = true; //第一帧标志
double first_image_time;        //第一帧的时间戳
double last_image_time = 0;     //上一帧的时间戳
int pub_count = 1;              //发布帧计数
bool init_pub = 0;              //是否发布第一帧
int MAX_CNT = 0;                //最大特征点数,在launch文件中设置

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 判断是否为第一帧
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec() / 1000;
        last_image_time = img_msg->header.stamp.toSec() / 1000;
        return;
    }

    // 判断图像数据流是否稳定，如果有帧错乱情况，则restart
    if(img_msg->header.stamp.toSec()/1000-last_image_time>1.0||img_msg->header.stamp.toSec()/1000<last_image_time)
    {
        ROS_WARN("图像不连续! 重置feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        restart_pub.publish(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.toSec() / 1000;
    // 控制光流跟踪的频率
    // round(发布的帧数/时间) <= 频率，可以发布
    if(round(1.0*pub_count/(img_msg->header.stamp.toSec()/1000-first_image_time))<=FREQ)
    {
        PUB_THIS_FRAME = true;
        // 如果频率很接近设置的频率了,更新起始时刻时间,并将发布帧数置0
        if(abs(1.0*pub_count/(img_msg->header.stamp.toSec()/1000-first_image_time)-FREQ)<0.01*FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec() / 1000;
            pub_count = 0;
        }
    }
    else 
        PUB_THIS_FRAME = false;

    // 创建常量共享图像智能指针,将ROS图像转为CV::Mat,并返回灰度图给ptr
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;

    // 计时开始正式处理图像
    TicToc t_whole_feature_tracker;
    trackerData.readImage(ptr->image, img_msg->header.stamp.toSec() / 1000);

    // 处理完成,开始发布数据,发布的是cur帧,即上一帧;因为如果发布当前帧forw,包含新特征点,如果在下一帧中没追踪到,会带来无意义的计算
    if(PUB_THIS_FRAME)
    {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud); //创建共享智能指针
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 track_cnt_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;

        //封装特征点的消息头
        feature_points->header = img_msg->header;   
        feature_points->header.frame_id = "camera";  //TODO 点云信息是否要改为相机系下,VINS-MONO是world

        for (uint i = 0; i < trackerData.ids.size();i++)
        {
            if(trackerData.track_cnt[i]>1)  //HACK 只显示多次跟踪的点,不显示新点吗
            {
                //封装特征点的points[]
                geometry_msgs::Point32 p;
                p.x = trackerData.cur_un_pts[i].x;
                p.y = trackerData.cur_un_pts[i].y;
                p.z = 1;
                feature_points->points.push_back(p);    //在setmask中已经把跟踪次数多的点给排序到前面了
                //封装特征点的channels[]
                id_of_point.values.push_back(trackerData.ids[i]);                   //记录点从起始的全局ID
                track_cnt_of_point.values.push_back(trackerData.track_cnt[i]);      //记录点的跟踪次数
                velocity_x_of_point.values.push_back(trackerData.pts_velocity[i].x);//点的x速度
                velocity_y_of_point.values.push_back(trackerData.pts_velocity[i].y);//点的y速度
                u_of_point.values.push_back(trackerData.cur_pts[i].x);              //点的像素坐标
                v_of_point.values.push_back(trackerData.cur_pts[i].y);              //点的像素坐标
            }
        }
        //封装特征点的channels[]
        id_of_point.name = "id_of_point";
        track_cnt_of_point.name = "track_cnt_of_point";
        velocity_x_of_point.name = "velocity_x_of_point";
        velocity_y_of_point.name = "velocity_y_of_point";
        u_of_point.name = "u_of_point";
        v_of_point.name = "v_of_point";
        
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(track_cnt_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);



        //发布特征点,第一帧默认不发布
        if(!init_pub)
            init_pub = 1;
        else
            feature_points_pub.publish(feature_points);
        ROS_INFO("-->发布当前帧: Time is : %.3f", feature_points->header.stamp.toSec() / 1000);
    }

    // 发布带特征点的图像
    if (SHOW_TRACK)
    {
        ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
        cv::Mat tmp_img = ptr->image; // cv::Mat的等号运算符是浅拷贝,相当于指针仍指向ptr->image
        for (uint i = 0; i < trackerData.cur_pts.size(); i++)
        {
            if (trackerData.track_cnt[i] <= 5) // 新点用绿色框显示
            {
                cv::circle(tmp_img, trackerData.cur_pts[i], 1, cv::Scalar(0, 255, 0), 2);
                cv::rectangle(tmp_img, cv::Rect(trackerData.cur_pts[i].x - 6, trackerData.cur_pts[i].y - 6, 12, 12), cv::Scalar(0, 255, 0), 1);
            }
            else if (trackerData.track_cnt[i] > 5 && trackerData.track_cnt[i] <= 30)
                cv::circle(tmp_img, trackerData.cur_pts[i], 1, cv::Scalar(0, 255, 0), 2); // 绿色
            else if (trackerData.track_cnt[i] > 30 && trackerData.track_cnt[i] <= 55)
                cv::circle(tmp_img, trackerData.cur_pts[i], 1, cv::Scalar(255, 0, 0), 2); // 蓝色
            else if (trackerData.track_cnt[i] > 55 && trackerData.track_cnt[i] <= 80)
                cv::circle(tmp_img, trackerData.cur_pts[i], 1, cv::Scalar(255, 0, 255), 2); // 紫色
            else
                cv::circle(tmp_img, trackerData.cur_pts[i], 1, cv::Scalar(0, 0, 255), 2); // 红色

            char name[10]; // 显示跟踪次数
            sprintf(name, "%d", trackerData.track_cnt[i]);
            cv::putText(tmp_img, name, trackerData.cur_pts[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
        }
        // opencv显示
        if(OPENCV_SHOW)
        {
        cv::Mat resize_tmp_img;
        cv::resize(tmp_img, resize_tmp_img, cv::Size(1920,1440));
        cv::imshow("tmp", resize_tmp_img);
        cv::waitKey(5);
        }
        feature_img_pub.publish(ptr->toImageMsg());
    }

    ROS_INFO("-->总特征跟踪时间为: %fms", t_whole_feature_tracker.toc());
    ROS_INFO("======================================================");
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "feature_tracker_node");
    ros::NodeHandle n;
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);

    boost::filesystem::path path = boost::filesystem::current_path();
    string CAMERA_INTRINSICS_PATH = ros::param::param("/feature_tracker_node/CAMERA_INTRINSICS_PATH", string("/home/speike/v_riekf_ws/src/feature_tracker/config/camera.yaml"));
    MAX_CNT = ros::param::param("/feature_tracker_node/MAX_CNT", 30);
    trackerData.readIntrinsicParameter(CAMERA_INTRINSICS_PATH);

    ros::Subscriber mono_image_sub = n.subscribe<sensor_msgs::Image>(IMAGE_TOPIC, 1000, img_callback);
    //发布feature_tracker的状态->是否重启
    restart_pub = n.advertise<std_msgs::Bool>("/feature_tracker/restart_pub", 10);
    //发布跟踪的特征点,2d点云形式,目前实际上是发布(u,v,1)//TODO 发布归一化坐标
    feature_points_pub = n.advertise<sensor_msgs::PointCloud>("/feature_tracker/feature_points_pub", 1000);
    //发布带跟踪特征点的图像
    feature_img_pub = n.advertise<sensor_msgs::Image>("/feature_tracker/feature_img_pub", 1000);

    ros::spin();
    return 0;
}
