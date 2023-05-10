#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

// 判断跟踪到的特征点是否在图像边界内
bool inBorder(const cv::Point2f &pt)
{
    int img_x = cvRound(pt.x); // 返回四舍五入的整型像素坐标
    int img_y = cvRound(pt.y);
    return img_x >= BORDER_SIZE && img_y >= BORDER_SIZE && img_x < COL - BORDER_SIZE && img_y < ROW - BORDER_SIZE;
}

// 根据status去除无法跟踪的特征点
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// 去除无法追踪到的特征点
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    cur_time = _cur_time;

    // 进行自适应直方图均衡化应对太亮或者太暗的情况
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_clahe;
        clahe->apply(_img, img);
        ROS_DEBUG("自适应直方图均衡化耗时: %fms", t_clahe.toc());
    }
    else
        img = _img;

    // 如果forw_img为空,说明第一次读入图像,用第一帧初始化全部;如果不为空,把当前接收帧给forw_img
    if (forw_img.empty())
        prev_img = cur_img = forw_img = img;
    else
        forw_img = img;

    // 清空forw_pts中的特征点,为本次光流做准备,因为这里储存的实际上是上一帧图像的特征点.
    forw_pts.clear(); // TODO:为什么不放到函数末尾清空为下次作准备

    // 如果当前处理帧中特征点不为空,使用光流法在forw中追踪cur中的特征点
    if (cur_pts.size() > 0)
    {
        TicToc t_optical_flow;
        vector<uchar> status;
        vector<float> err;

        // 进行光流追踪!!!
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        float max_err = *max_element(err.begin(), err.end());
        float avg_err = accumulate(err.begin(), err.end(), 0.0) / err.size();

        // 剔除位于图像边界外的点,如果跟踪成功且不在图像边界内,标记为0
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && (!inBorder(forw_pts[i]) || err[i] > (max_err + avg_err)/2))
                status[i] = 0;

        int size_old = cur_pts.size();
        // 根据status, 把跟踪失败的点剔除  //TODO：是否要把跟踪失败情况发给RIEKF模块进行点去除
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);

        ROS_DEBUG("LK光流追踪: %d -> %lu : %.2f%%, 耗时：%fms", size_old, forw_pts.size(), (100.0 * forw_pts.size() / size_old), t_optical_flow.toc());
    }

    // 对追踪成功的每个点追踪次数+1
    for (auto &n : track_cnt)
        n++;

    // 如果需要发布特征点
    if (PUB_THIS_FRAME)
    {
        // 通过基础矩阵F剔除外点
        rejectWithF();
        // 相邻特征点间隔30个像素设置mask
        setMask();

        // 计算是否需要增加特征点
        if ((MAX_CNT - static_cast<int>(forw_pts.size())) > 0)
        {
            addPoints();
        }
        else
            n_pts.clear();
    }

    // 更新特征点ID
    updateID();

    // 更新图像
    prev_img = cur_img;
    cur_img = forw_img;

    // 更新特征点
    prev_pts = cur_pts;
    cur_pts = forw_pts;
    prev_un_pts = cur_un_pts;

    // 计算归一化坐标和速度
    undistortedPoints();
    
    // 更新时间
    prev_time = cur_time;

    //TODO 是否需要去畸变矫正计算速度
}

void FeatureTracker::rejectWithF()
{
    if(forw_pts.size()>=8)  //8点法求基础矩阵
    {
        TicToc t_rejectWithF;

        vector<cv::Point2f> un_cur_pts(cur_pts.size()); //归一化平面坐标
        vector<cv::Point2f> un_forw_pts(forw_pts.size());
        for (uint i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;

            // 根据相机内参由像素坐标得到去畸变后的归一化坐标
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            // 然后用归一化坐标再转回像素坐标,此时图像不受畸变的影响
             tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
             tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_old = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);

        ROS_DEBUG("基础矩阵F剔除外点: %d -> %lu : %.2f%%, 耗时：%fms", size_old, forw_pts.size(), (100.0 * forw_pts.size() / size_old), t_rejectWithF.toc());
    }
}

/**
 * @brief   对forw_pts中点按跟踪次数排序,优先在跟踪次数多的点处设置mask去除密集点
 */
void FeatureTracker::setMask()
{
    TicToc t_mask;

    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255)); //设置全白灰度图

    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id; //(追踪次数,特征点,特征点id)
    for (uint i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i]))); // HACK 为什么不直接用forw_pts的序号作为点id

    //根据跟踪次数对forw_pts中的特征点排序 //TODO 学习lambda表达式
    sort(cnt_pts_id.begin(), cnt_pts_id.end(),
        [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
        {
            return a.first > b.first;
        });

    // 清空三个容器方便下面按照排序完后的方法重新存入
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    // 根据排序,优先在跟踪次数多的点处设置mask,然后重新存入三个容器
    for(auto &it:cnt_pts_id)
    {
        if(mask.at<uchar>(it.second.first)==255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }

    ROS_DEBUG("设置Mask矩阵耗时: %fms", t_mask.toc());
}

/**
 * @brief   提取新特征点,向forw_pts中新增特征点并初始化点ids和追踪次数track_cnt
*/
void FeatureTracker::addPoints()
{
    TicToc t_detect_new_feature;

    if (mask.empty()) // 检查有没有提前设置好mask
        cout << "mask is empty " << endl;
    if (mask.type() != CV_8UC1)
        cout << "mask type wrong " << endl;
    if (mask.size() != forw_img.size())
        cout << "wrong size " << endl;

    int size_old = forw_pts.size();     //增加前点的数量
    
    // 算法选择, 1:Shi-Tomas角点亚像素级别检测+光流法,
    // 3.AKAZE特征检测器,FLANN特征匹配器+光流法)
    if(FeatureClass==1)
    {
        // TODO 尝试更改提取点的质量(0.01~1)
        cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.05, MIN_DIST, mask);
        if (n_pts.size()>0)
            cv::cornerSubPix(forw_img, n_pts, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 40, 0.01));
    }
    else if (FeatureClass==2)
    {
        // BUG:未实现,暂时不用
        // 将mask矩阵取反
        cv::Mat mask_inv;
        cv::bitwise_not(mask, mask_inv);
        akaze->detect(forw_img, n_pts_akaze, mask);
        // 根据特征点的响应值排序
        sort(n_pts_akaze.begin(), n_pts_akaze.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b) { return a.response > b.response; });
        // 将AKAZE特征点转换为cv::Point2f格式
        for (auto &p : n_pts_akaze)
            n_pts.push_back(p.pt);
        n_pts_akaze.clear();
        // 取n_pts中前MAX_CNT-forw_pts.size()个点
        n_pts.resize(min(MAX_CNT - forw_pts.size(), n_pts.size()));
        if (n_pts.size()>0)
            cv::cornerSubPix(forw_img, n_pts, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 40, 0.01));
    }

    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);      // 新特征点id初始化为-1
        track_cnt.push_back(1); // 新特征点跟踪次数初始化为1
    }
    // n_pts.clear();

    ROS_WARN("提取新特征点: %d -> %lu : %.2f%%, 耗时：%fms", size_old, forw_pts.size(), (100.0 * forw_pts.size() / size_old), t_detect_new_feature.toc());
}


void FeatureTracker::updateID()
{
    for (int i = 0; i < ids.size(); i++)
        if (ids[i] == -1)
            ids[i] = n_id++;
    ROS_DEBUG("总特征点数量(n_id): %d", n_id);
}

/**
 * @brief 读取相机内参
*/
void FeatureTracker::readIntrinsicParameter(const string CAMERA_INTRINSICS_PATH)
{
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(CAMERA_INTRINSICS_PATH);
}

/**
 * @brief 对角点图像坐标进行去畸变矫正，转换到归一化坐标系上，并计算每个角点的速度。
*/
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear(); 
    // 计算归一化像素坐标
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;

        //根据不同的相机模型将像素坐标转化为去畸变后的归一化坐标
        m_camera->liftProjective(a, b);

        //再延伸到深度归一化平面上
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
/*         ROS_WARN_STREAM(left 
                        << "u: " << setw(10) << a.x() 
                        << "v: " << setw(10) << a.y() 
                        << "b.x: " << setw(12) << b.x()
                        << "b.y: " << setw(12) << b.y()
                        << "b.z: " << setw(12) << b.z()); */
    }
    // 计算每个特征点的速度
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }        
    }
    prev_un_pts_map = cur_un_pts_map;   //TODO map貌似没用
}
