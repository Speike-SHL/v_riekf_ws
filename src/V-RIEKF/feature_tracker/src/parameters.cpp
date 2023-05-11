#include "parameters.h"

int ROW = 480;            // 图像高度
int COL = 640;            // 图像宽度
int FOCAL_LENGTH = 443;   // 焦距
const int NUM_OF_CAM = 1; // 相机数目

std::string IMAGE_TOPIC = "/origin/mono_image_pub";        // 图像的ROS TOPIC
std::string IMU_TOPIC = "/origin/imu_measure_pub";         // IMU的ROS TOPIC
int MIN_DIST = 30;          // 特征点之间的最小间隔,按照此间隔设置mask矩阵
const int BORDER_SIZE = 2;  // 特征点距离图像边界的像素大小,想让特征点远离边界可以调大
int FREQ = 25;              // 控制图像光流跟踪的频率, 最小为10hz, 才能有较好的效果
double F_THRESHOLD = 0.5;   // ransac算法的门限,如果超过这个像素值,就认为匹配点失败,实际上为外点,无法计算F
int EQUALIZE = 1;           // 如果光太亮或太暗则为1，进行直方图均衡化
int FeatureClass = 1;       // 特征点提取与匹配算法策略(1:Shi-Tomasi角点亚像素级别检测+光流法, 2.AKAZE特征检测器,FLANN特征匹配器+光流法) // BUG:未实现,暂时不用2

/*----------------------------------------显示设置---------------------------------------*/
int SHOW_TRACK = 2;         // 0：不发布特征点图像，1：发布特征点图像，2：发布左右两帧光流匹配图像
bool PUB_THIS_FRAME = false;// 是否发布当前帧特征点, 默认为false, 程序中会根据设置的发布频率对其修改
bool OPENCV_SHOW = false;   // 是否打开opencv显示带特征点的视频, 会导致有5ms的计算延时
