/* ----------------------------------------------------------------------------
 * Copyright 2023, Speike <shao-haoluo@foxmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file       depth_recovery_node.cpp
 * @author     Speike
 * @date       2023/04/20 19:59:09
**/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
using namespace std;

//储存深度图的时间戳和共享智能指针
map<double, sensor_msgs::ImageConstPtr> depth_img_map;
ros::Publisher feature_points_depth_pub;
// 深度相机最近距离
double MIN_RANGE;
// 深度相机最远距离
double MAX_RANGE;

/**
 * @brief 将深度图时间戳和共享智能指针储存,在feature_callback中找到对应帧的深度图后,
 * 清空map容器
*/
void depth_image_callback(const sensor_msgs::ImageConstPtr &depth_img)
{
    depth_img_map.insert(make_pair(depth_img->header.stamp.toSec() / 1000, depth_img));
}

/**
 * @brief 从容器中找到对应帧的深度图,获取深度信息发布,然后清空map容器
*/
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_point)
{
    double time = feature_point->header.stamp.toSec() / 1000;
    // 查找当前帧对应的深度图
    auto pos = depth_img_map.find(time);
    if(pos != depth_img_map.end())
    {
        cv_bridge::CvImageConstPtr ptr;
        ptr = cv_bridge::toCvCopy((*pos).second, sensor_msgs::image_encodings::MONO8);
        cv::Mat depth_img = ptr->image; //拿到当前帧深度图(宽640,高480,单通道)
        sensor_msgs::ChannelFloat32ConstPtr u_channel_ptr;
        sensor_msgs::ChannelFloat32ConstPtr v_channel_ptr;
        sensor_msgs::ChannelFloat32 Z_of_point;
        sensor_msgs::PointCloudPtr feature_points_Z(new sensor_msgs::PointCloud);
        // 遍历当前帧的channels,找到u_of_point和v_of_point对应的channel
        for (int i = 0; i < feature_point->channels.size(); i++)
        {
            if(feature_point->channels[i].name == "u_of_point")
                u_channel_ptr = boost::make_shared<const sensor_msgs::ChannelFloat32>(feature_point->channels[i]);
            else if(feature_point->channels[i].name == "v_of_point")
                v_channel_ptr = boost::make_shared<const sensor_msgs::ChannelFloat32>(feature_point->channels[i]);
        }
        // 遍历(u,v),得到Z
        for (int i = 0; i < u_channel_ptr->values.size(); i++)
        {
            int u = cvRound(u_channel_ptr->values[i]);
            int v = cvRound(v_channel_ptr->values[i]);
            if(u>0 && u<depth_img.cols && v>0 && v<depth_img.rows)
            {
                // 获取像素值
                int Z = static_cast<int>(depth_img.ptr<uchar>(v)[u]);
                // 超出范围距离设置为0,数据不能用
                if(Z==255)
                    Z_of_point.values.push_back(0);
                // 否则将0～255像素值转化为真实距离
                else if(Z>0 && Z<255)
                    Z_of_point.values.push_back(static_cast<double>(Z)*(MAX_RANGE-MIN_RANGE)/255.0+0.01);
            }
            else
            {
                ROS_ERROR("depth_recovery_node.cpp访问像素越界!!!");
            }
        }
        // 封装带深度的点云数据
        feature_points_Z->header = feature_point->header;
        feature_points_Z->points = feature_point->points;
        feature_points_Z->channels = feature_point->channels;
        Z_of_point.name = "Z_of_point";
        feature_points_Z->channels.push_back(Z_of_point);
        feature_points_depth_pub.publish(feature_points_Z);
        // 清除当前帧使用过的深度图
        depth_img_map.erase(pos);
        ROS_DEBUG_STREAM(depth_img_map.size());
    }
    else
    {
        ROS_WARN("找不到帧%.3f对应的深度图!",feature_point->header.stamp.toSec()/1000);
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "depth_recovery_node");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    MAX_RANGE = ros::param::param("/depth_recovery_node/MAX_RANGE", 8);
    MIN_RANGE = ros::param::param("/depth_recovery_node/MIN_RANGE", 0.01);
    // 订阅深度图
    ros::Subscriber depth_image_sub = nh.subscribe<sensor_msgs::Image>("/origin/depth_image_pub", 2000, depth_image_callback);
    //订阅(u,v,1)
    ros::Subscriber feature_points_sub = nh.subscribe<sensor_msgs::PointCloud>("/feature_tracker/feature_points_pub", 2000, feature_callback);
    //发布(u,v,1),Z
    feature_points_depth_pub = nh.advertise<sensor_msgs::PointCloud>("/depth_recovery/feature_points_depth_pub", 1000);
    ros::spin();
    return 0;
}
