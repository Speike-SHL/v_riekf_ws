#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <numeric>
#include "parameters.h"
#include "tic_toc.h"

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

using namespace std;

extern int MAX_CNT;  //最大特征点数,在launch文件中设置

//---------------------- 不在类中的函数 ----------------------
bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
    //-------------------------- 类方法 ----------------------------
    FeatureTracker() = default;

    void readImage(const cv::Mat &_img, double _cur_time);

    void rejectWithF();

    void setMask();

    void addPoints();

    void updateID();

    void readIntrinsicParameter(const string CAMERA_INTRINSICS_PATH);

    void undistortedPoints();

    //-------------------------- 类属性 ----------------------------
    double cur_time;    // 当前时间
    double prev_time;   //上次时间

    cv::Mat prev_img;   //上一帧(处理过的帧)图像
    cv::Mat cur_img;    //当前帧(当前处理帧)图像
    cv::Mat forw_img;   //下一帧(当前接收帧)图像
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;        //上一帧(prev),当前帧(cur),下一帧(forw)特征点(points)
    vector<cv::Point2f> prev_un_pts, cur_un_pts;            //去畸变后的归一化坐标
    map<int, cv::Point2f> prev_un_pts_map, cur_un_pts_map;  //<id, 去畸变后的归一化坐标>
    vector<cv::Point2f> pts_velocity;                       //当前帧相对前一帧特征点沿x,y方向的像素移动速度
    vector<int> ids;            // 能够被追踪到的特征点ID
    vector<int> track_cnt;      //当前帧forw_img中每个特征点被追踪的次数

    cv::Mat mask;               //图像掩膜
    vector<cv::Point2f> n_pts;  //从forw_img中新提取的特征点

    //静态id,用于updataID中更新全局ID,每检测到一个新的特征点,就将n_id作为该特征点的id，然后n_id加1
    //n_id会一直增加,可能到上千上万,代表从初始到最终提取过的所有点
    static int n_id;

    camodocal::CameraPtr m_camera;  //创建相机模型

    //---------------------- AKAZE+FLANN+光流 -----------------------
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(); // AKAZE特征提取器
    cv::Ptr<cv::FlannBasedMatcher> flann = cv::FlannBasedMatcher::create();  //FLANN匹配器
    cv::Mat prev_des,cur_des,forw_des;  //描述子
    vector<cv::KeyPoint> n_pts_akaze;  //从forw_img中使用akaze新提取的特征点
};
