/* ----------------------------------------------------------------------------
 * Copyright 2023, Speike <shao-haoluo@foxmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file       origin_data.cpp
 * @author     Speike
 * @date       2023/04/14 13:48:15
 * @brief      读取火星无人机仿真数据集，并创建origin_data_node节点发布数据
**/

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::init(argc, argv, "origin_data_node");
    ros::NodeHandle nh;
    //创建五个发布者
    ros::Publisher imu_measure_pub = nh.advertise<sensor_msgs::Imu>("/origin/imu_measure_pub", 100);    
    ros::Publisher ground_truth_pub = nh.advertise<nav_msgs::Odometry>("/origin/ground_truth_pub", 100);
    ros::Publisher ground_truth_path_pub = nh.advertise<nav_msgs::Path>("/origin/ground_truth_path_pub", 100);
    ros::Publisher mono_image_pub = nh.advertise<sensor_msgs::Image>("/origin/mono_image_pub", 1000);
    ros::Publisher depth_image_pub = nh.advertise<sensor_msgs::Image>("/origin/depth_image_pub", 1000);

    string DataSetPath = ros::param::param("/origin_data_node/DataSetPath", string("/media/speike/Data/DataSet/Data/imu_noise"));
    int Rate = ros::param::param("/origin_data_node/Rate", 125);
    string End_Flag = ros::param::param("/origin_data_node/End_Flag", string("60.32800000"));

    ifstream infile_imu(string(DataSetPath)+string("/imu_measure.txt"));
    ifstream infile_ground_truth(string(DataSetPath)+string("/ground_truth.txt"));
    string mono_pic_path = string(DataSetPath) + string("/mono_pic/");
    string depth_pic_path = string(DataSetPath) + string("/depth_pic/");
    string line;

    int count = 0;
    string t;
    nav_msgs::Path ground_truth_path_msg;   //循环外创建path对象，用于保存以往路径点

    ros::Rate loop_rate_fast(Rate);
    ros::Duration(5.0).sleep();
    while (ros::ok())
    {
        if(t == End_Flag) 
            ros::shutdown();
        count++;
        // 打包发布IMU数据
        sensor_msgs::Imu imu_measure_msg;
        std::getline(infile_imu, line);
        vector<string> measurement;
        boost::split(measurement,line,boost::is_any_of(" "));
        if(measurement.size()==7)
        {
            imu_measure_msg.header.stamp = ros::Time(stof(measurement[0]) * 1000);
            imu_measure_msg.header.frame_id = "imu";
            imu_measure_msg.angular_velocity.x = stod(measurement[1]);
            imu_measure_msg.angular_velocity.y = stod(measurement[2]);
            imu_measure_msg.angular_velocity.z = stod(measurement[3]);
            imu_measure_msg.linear_acceleration.x = stod(measurement[4]);
            imu_measure_msg.linear_acceleration.y = stod(measurement[5]);
            imu_measure_msg.linear_acceleration.z = stod(measurement[6]);
            imu_measure_msg.angular_velocity_covariance[0] = 0.25;
            imu_measure_msg.angular_velocity_covariance[4] = 0.25;
            imu_measure_msg.angular_velocity_covariance[8] = 0.25;
            imu_measure_msg.linear_acceleration_covariance[0] = 0.25;
            imu_measure_msg.linear_acceleration_covariance[4] = 0.25;
            imu_measure_msg.linear_acceleration_covariance[8] = 0.25;
            imu_measure_pub.publish(imu_measure_msg);
            t = measurement[0];
        }

        // 打包Ground_Truth数据
        geometry_msgs::PoseStamped poses;
        nav_msgs::Odometry ground_truth_msg;
        std::getline(infile_ground_truth, line);
        boost::split(measurement,line,boost::is_any_of(" "));
        if(measurement.size()==11)
        {
            ground_truth_msg.header.stamp = ros::Time(stof(measurement[0]) * 1000);    
            ground_truth_msg.header.frame_id = "world";
            ground_truth_msg.pose.pose.orientation.w = stod(measurement[1]);
            ground_truth_msg.pose.pose.orientation.x = stod(measurement[2]);
            ground_truth_msg.pose.pose.orientation.y = stod(measurement[3]);
            ground_truth_msg.pose.pose.orientation.z = stod(measurement[4]);
            ground_truth_msg.twist.twist.linear.x = stod(measurement[5]);
            ground_truth_msg.twist.twist.linear.y = stod(measurement[6]);
            ground_truth_msg.twist.twist.linear.z = stod(measurement[7]);
            ground_truth_msg.pose.pose.position.x = stod(measurement[8]);
            ground_truth_msg.pose.pose.position.y = stod(measurement[9]);
            ground_truth_msg.pose.pose.position.z = stod(measurement[10]);
            ground_truth_pub.publish(ground_truth_msg); //发布grond_truth

            ground_truth_path_msg.header.stamp = ros::Time(stof(measurement[0]) * 1000);
            ground_truth_path_msg.header.frame_id = "world";
            poses.header.frame_id = "world"; // 当前帧路径点打包
            poses.pose.orientation.w = stod(measurement[1]);
            poses.pose.orientation.x = stod(measurement[2]);
            poses.pose.orientation.y = stod(measurement[3]);
            poses.pose.orientation.z = stod(measurement[4]);
            poses.pose.position.x = stod(measurement[8]);
            poses.pose.position.y = stod(measurement[9]);
            poses.pose.position.z = stod(measurement[10]);
            ground_truth_path_msg.poses.push_back(poses);
            ground_truth_path_pub.publish(ground_truth_path_msg);   //发布path
        }

        // 打包mono和depth图像数据
        if(count==5)
        {
            string pic_name = measurement[0].erase(measurement[0].find(".") + 3, 5) + ".jpg";
            string mono_image_name = mono_pic_path + pic_name;
            string depth_image_name = depth_pic_path + pic_name;
            cv::Mat mono_image = cv::imread(mono_image_name, cv::IMREAD_COLOR);
            cv::Mat depth_image = cv::imread(depth_image_name, cv::IMREAD_GRAYSCALE);

            cv_bridge::CvImage cv_image_mono;   //打包mono数据
            cv_image_mono.encoding = "bgr8";
            cv_image_mono.image = mono_image;
            cv_image_mono.header.stamp = ros::Time(stof(t) * 1000);
            cv_image_mono.header.frame_id = "camera";
            sensor_msgs::Image ros_image_mono;
            ros_image_mono.width = 640;
            ros_image_mono.height = 480;
            cv_image_mono.toImageMsg(ros_image_mono);
            mono_image_pub.publish(ros_image_mono);

            cv_bridge::CvImage cv_image_depth;  //打包depth数据
            cv_image_depth.encoding = "mono8";
            cv_image_depth.image = depth_image;
            cv_image_depth.header.stamp = ros::Time(stof(t) * 1000);
            cv_image_depth.header.frame_id = "camera";
            sensor_msgs::Image ros_image_depth;
            ros_image_depth.width = 640;
            ros_image_depth.height = 480;
            cv_image_depth.toImageMsg(ros_image_depth);
            depth_image_pub.publish(ros_image_depth);
            count = 0;
        }
        
        //整数秒输出t
        ROS_DEBUG_STREAM("\033[1;36m" << "Time is : " << t << "\033[0m");
        loop_rate_fast.sleep();
        ros::spinOnce();
    }
    return 0;
}
