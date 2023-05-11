#pragma once
#include <ros/ros.h>

extern int ROW;                 // 图像宽度
extern int COL;                 // 图像高度
extern int FOCAL_LENGTH;        // 焦距
extern const int NUM_OF_CAM;    // 相机的个数

extern std::string IMAGE_TOPIC;             // 图像的ROS TOPIC
extern std::string IMU_TOPIC;               // IMU的ROS TOPIC
extern int MIN_DIST;            // 特征点之间的最小间隔
extern const int BORDER_SIZE;   // 特征点距离图像边界的像素大小,想让特征点远离边界可以调大
extern int FREQ;                // 控制图像光流跟踪的频率, 最小为10hz, 才能有较好的效果
extern double F_THRESHOLD;      // ransac算法的门限
extern int EQUALIZE;            // 如果光太亮或太暗则为1，进行直方图均衡化
extern int FeatureClass;        /// 特征点提取与匹配算法策略(1:Shi-Tomasi角点亚像素级别检测+光流法, 2.AKAZE特征检测器,FLANN特征匹配器+光流法)
/*----------------------------------------显示设置---------------------------------------*/
extern int SHOW_TRACK;          // 0：不发布特征点图像，1：发布特征点图像，2：发布左右两帧光流匹配图像
extern bool PUB_THIS_FRAME;     // 是否发布当前帧特征点, 默认为false, 程序中会根据设置的发布频率对其修改
extern bool OPENCV_SHOW;        // 是否打开opencv显示带特征点的视频, 会导致有5ms的计算延时
