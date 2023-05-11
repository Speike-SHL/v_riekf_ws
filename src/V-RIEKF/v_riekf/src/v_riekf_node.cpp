/* ----------------------------------------------------------------------------
 * Copyright 2023, Speike <shao-haoluo@foxmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file       v_riekf_node.cpp
 * @author     Speike
 * @date       2023/04/20 19:58:19
**/

#include <ros/ros.h>
#include <iomanip>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include "InEKF/InEKF.h"
#include "tic_toc.h"

using namespace std;
using namespace inekf;

#define DT_MIN 1e-6
#define DT_MAX 1

ros::Publisher riekf_esti_pub;        // 发布状态估计结果
ros::Publisher riekf_esti_path_pub;   // 发布状态估计轨迹
nav_msgs::Path riekf_esti_path_msg;   // 创建全局path,保存以往路径点
InEKF filter; //初始化滤波器对象
Eigen::Matrix<double, 6, 1> imu_measurement = Eigen::Matrix<double, 6, 1>::Zero();
Eigen::Matrix<double, 6, 1> imu_measurement_prev = Eigen::Matrix<double, 6, 1>::Zero();
Eigen::Matrix3d K, K_inv;
Eigen::Matrix3d R_bc;
Eigen::Matrix3d R_;     // 将理论上的坐标(Z轴指向图像)转化为实际无人机上的坐标(X轴指向图像)
Eigen::Vector3d p_bc = Eigen::Vector3d::Zero();
double t = 0;
double t_prev = 0;

void pub_state()
{
    //XXX 显示状态估计结果
    cout << filter.getState() << endl;  
    geometry_msgs::PoseStamped poses;
    nav_msgs::Odometry riekf_esti_msg;
    // 获取当前状态
    Eigen::Matrix3d R = filter.getState().getRotation().eval();
    Eigen::Vector3d v = filter.getState().getVelocity().eval();
    Eigen::Vector3d p = filter.getState().getPosition().eval();
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(R);
    quaternion.normalize();
    // 封装估计值
    riekf_esti_msg.header.stamp = ros::Time(t * 1000);
    riekf_esti_msg.header.frame_id = "world";
    riekf_esti_msg.pose.pose.orientation.w = quaternion.w();
    riekf_esti_msg.pose.pose.orientation.x = quaternion.x();
    riekf_esti_msg.pose.pose.orientation.y = quaternion.y();
    riekf_esti_msg.pose.pose.orientation.z = quaternion.z();
    riekf_esti_msg.pose.pose.position.x = p[0];
    riekf_esti_msg.pose.pose.position.y = p[1];
    riekf_esti_msg.pose.pose.position.z = p[2];
    riekf_esti_msg.twist.twist.linear.x = v[0];
    riekf_esti_msg.twist.twist.linear.y = v[1];
    riekf_esti_msg.twist.twist.linear.z = v[2];
    riekf_esti_pub.publish(riekf_esti_msg);
    // 封装估计路径
    riekf_esti_path_msg.header.stamp = ros::Time(t * 1000);
    riekf_esti_path_msg.header.frame_id = "world";
    poses.header.frame_id = "world";
    poses.pose.orientation.w = quaternion.w();
    poses.pose.orientation.x = quaternion.x();
    poses.pose.orientation.y = quaternion.y();
    poses.pose.orientation.z = quaternion.z();
    poses.pose.position.x = p[0];
    poses.pose.position.y = p[1];
    poses.pose.position.z = p[2];
    riekf_esti_path_msg.poses.push_back(poses);
    riekf_esti_path_pub.publish(riekf_esti_path_msg);
}

void feature_points_depth_callback(const sensor_msgs::PointCloudConstPtr &feature_point)
{
    TicToc t_landmark;
    t = feature_point->header.stamp.toSec() / 1000;
    vectorLandmarks measured_landmarks;
    if(t<0)
        ROS_DEBUG("忽略当前帧=");
    else
    {
        // 循环读取地标
        for (int i = 0; i < feature_point->points.size(); i++)
        {
            Eigen::Vector3d point_XY1_norm; //归一化坐标
            double X_norm = feature_point->points[i].x;
            double Y_norm = feature_point->points[i].y;
            double Z;
            int id;
            for (int j = 0; j < feature_point->channels.size(); j++)
            {
                if(feature_point->channels[j].name == "Z_of_point")
                    Z = feature_point->channels[j].values[i];
                else if(feature_point->channels[j].name == "id_of_point")
                    id = feature_point->channels[j].values[i];
            }
            if(Z==0)    //忽略
                continue;
            else
            {
                point_XY1_norm << X_norm, Y_norm, 1.0;
                Eigen::Vector3d p_bl = R_bc * R_ * Z * point_XY1_norm; //body系下像素坐标
                Landmark landmark(id, p_bl);
                measured_landmarks.push_back(landmark);
            }
        }
        ROS_DEBUG("当前帧可用地标数目:%ld", measured_landmarks.size());
        // 使用地标进行更新，更新不需要知道t
        filter.CorrectLandmarks(measured_landmarks);
        // 发布状态估计结果, 因为imu_measure_callback已经发布过了，这里只更新状态就行
        // pub_state();
    }
    ROS_INFO_STREAM("\033[1;35m"
                    << "Received LANDMARK observation, correcting state, Time is : " << setprecision(5) << t << ", 耗时: " << t_landmark.toc() << "ms"
                    << "\033[0m");
}
void imu_measure_callback(const sensor_msgs::ImuConstPtr &imu_measure)
{
    TicToc t_imu_measure;
    t = imu_measure->header.stamp.toSec() / 1000;
    imu_measurement << imu_measure->angular_velocity.x,
                       imu_measure->angular_velocity.y,
                       imu_measure->angular_velocity.z,
                       imu_measure->linear_acceleration.x,
                       imu_measure->linear_acceleration.y,
                       imu_measure->linear_acceleration.z;
    double dt = t - t_prev;
    // Propagate using IMU data
    if (dt > DT_MIN && dt < DT_MAX)
        //TODO 为什么不用当前帧的IMU数据
        filter.Propagate(imu_measurement, dt);
    // 发布状态估计结果
    pub_state();
    // Store previous timestamp
    t_prev = t;
    imu_measurement_prev = imu_measurement; 
    // ROS_INFO("Received IMU Data, propagating state, Time is : %.3f, 耗时: %fms", t, t_imu_measure.toc());   
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    /*--------------------------ROS节点处理初始化--------------------------*/
    ros::init(argc, argv, "v_riekf_node");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    double GyroNoise = ros::param::param<double>("/v_riekf_node/GyroNoise", 0.02);
    double AcceNoise = ros::param::param<double>("/v_riekf_node/AcceNoise", 0.02);
    double GyroBiasNoise = ros::param::param<double>("/v_riekf_node/GyroBiasNoise", 0);
    double AcceBiasNoise = ros::param::param<double>("/v_riekf_node/AcceBiasNoise", 0);
    double LandmarkNoise = ros::param::param<double>("/v_riekf_node/LandmarkNoise", 0.04);

    /*-----------------------------参数初始化----------------------------*/
    K << 443.1871, 0.0, 320.5185, 0.0, 443.1156, 240.1581, 0.0, 0.0, 1.0;
    K_inv = K.inverse();
    // R_bc << 0.7073883, 0.0, 0.7068252, 0.0, 1.0, 0.0, -0.7068252, 0.0, 0.7073883;   //FIXME 坐标变换
    R_bc << 0.0007963, 0.0, 0.9999997, 0.0, 1.0, 0.0, -0.9999997, 0.0, 0.0007963;
    R_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    /*---------------------------状态估计初始化---------------------------*/
    //  ---- Initialize invariant extended Kalman filter ----- //
    RobotState initial_state;

    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    R0 << 1, 0, 0, 0, 1, 0, 0, 0, 1;//初始化IMU旋转矩阵
    v0 << 0, 0, 0;                  //初始化速度
    p0 << 0, 0, 0;                  //初始化位置
    bg0 << 0, 0, 0;                 //初始化陀螺仪bias
    ba0 << 0, 0, 0;                 //初始化加速度计bias
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initialize state covariance
    NoiseParams noise_params;
    noise_params.setGyroscopeNoise(GyroNoise);
    noise_params.setAccelerometerNoise(AcceNoise);
    noise_params.setGyroscopeBiasNoise(GyroBiasNoise);
    noise_params.setAccelerometerBiasNoise(AcceBiasNoise);
    noise_params.setLandmarkNoise(LandmarkNoise);

    // Initialize filter
    filter.setState(initial_state);
    filter.setNoiseParams(noise_params);
    filter.setG(Eigen::Vector3d(0, 0, -3.65));  //FIXME 重力加速度
    // cout << "Noise parameters are initialized to: \n";
    // cout << filter.getNoiseParams() << endl;
    // cout << "Robot's state is initialized to: \n";
    // cout << filter.getState() << endl;


    //订阅(u,v,1),Z
    ros::Subscriber feature_points_depth_sub = nh.subscribe<sensor_msgs::PointCloud>("/depth_recovery/feature_points_depth_pub", 2000, feature_points_depth_callback);
    //订阅IMU
    ros::Subscriber imu_measure_sub = nh.subscribe<sensor_msgs::Imu>("/origin/imu_measure_pub", 200, imu_measure_callback);
    //发布状态估计结果
    riekf_esti_pub = nh.advertise<nav_msgs::Odometry>("/riekf/riekf_esti_pub", 100);
    riekf_esti_path_pub = nh.advertise<nav_msgs::Path>("/riekf/riekf_esti_path_pub", 100);
    ros::spin();
    return 0;
}
