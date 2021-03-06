#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <cstring>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
using namespace std;

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>

// 点云类型和点云指针类型重定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef pcl::PointXYZI PointT_I;
typedef pcl::PointCloud<PointT_I> PointCloud_I;

typedef pcl::PointXYZRGB PointT_C;
typedef pcl::PointCloud<PointT_C> PointCloud_C;

// PointXYZITS 自定义带有强度、时间戳、线号的三维点信息; 仅用于在驱动程序中读取更加详尽的信息
struct PointXYZITS{
    float x;
    float y;
    float z;
    float intensity;
    float timeStamp;
    int scanID;
};

// 里程计参数
// VLP-16 or RSLidar-16
const int N_SCAN = 32;
const int Horizon_SCAN = 1800;
const float ang_res_x = 0.2;      //水平方向上角度分辨率
const float ang_res_y = 2.0;      //竖直方向上角度分辨率
const int SCAN_PERIOD = 0.1; // 雷达扫描
// const float ang_bottom = 15.0+0.1;//雷达竖直方向角度区间[-15, 15],取底部角15，防止计算出现负数
// const int groundScanInd = 7;      //认为地面只会出现在0-7线这8条scan中

const float sensorMountAngle = 0.0; // 雷达倾斜角度
const float segmentTheta = 1.0472;  // segmentTheta=1.0472<==>60度,在imageProjection中用于判断平面
const int segmentValidPointNum = 5; // 聚类中有效点的最小数目
const int segmentValidLineNum = 3;  // 聚类中竖直方向有效线数的最小数目
const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
const float segmentAlphaY = ang_res_y / 180.0 * M_PI;

const int edgeFeatureNum = 2;
const int surfFeatureNum = 4;
const int sectionsTotal = 6;
const float edgeThreshold = 0.1;
const float surfThreshold = 0.1;
const float nearestFeatureSearchSqDist = 25;
// 特征点的平滑程度
struct Smoothness{
    float value;
    int   ind;
};

struct by_value{ 
    bool operator()(Smoothness const &left, Smoothness const &right) { 
        return left.value < right.value;
    }
};

// SegInfo点云分割的信息存储格式
struct SegInfo{
    std::vector<float> segmentedCloudRange;
    std::vector<int> segmentedCloudColInd;
    std::vector<int> startRingIndex;
    std::vector<int> endRingIndex;
    std::vector<int> segmentedCloudGroundFlag;
    SegInfo(){
        startRingIndex.resize(N_SCAN, 0);
        endRingIndex.resize(N_SCAN, 0);
        segmentedCloudRange.resize(N_SCAN * Horizon_SCAN, 0.0);
        segmentedCloudColInd.resize(N_SCAN * Horizon_SCAN, -1);
        segmentedCloudGroundFlag.resize(N_SCAN * Horizon_SCAN, 0);
    }
};
// InitParams 算法中的参数
struct InitParams{
    /**** 地面分割算法参数 *******/
    bool visualize_ground;          // 是否可视化估计地面
    double r_min_square;            // 距离雷达最近距离的平方，设定圆环的内环
    double r_max_square;            // 距离雷达最远距离的平方，设定圆环的外环
    int n_bins;                     // 每个seg中bin的个数
    int n_segments;                 // 根据绕z轴旋转角度划分的seg个数
    double max_dist_to_line;        // 距离地面线的最大距离，小于该距离被认为是地面
    double max_slope;               // 地面线的最大斜度(使用拟合的地面线代表该seg的情况)
    double max_error_square;        // 直线拟合的最大平方差
    double long_threshold;          // 两点间距离大于阈值时，被认为是互相远离的点
    double max_long_height;         // 点到线距离的最大高度差
    double max_start_height;        // 被标记为地面的线的起点最大高度
    double sensor_height;           // 雷达传感器距离地面的高度
    double line_search_angle;       // 在角度方向[rad]的范围内搜索直线（划定了扇形区域）
    int n_threads;                  // 局部加速的线程数目
    /**** 地平面矫正算法参数 ****/
    float max_deviation_deg;         //有精度地面在雷达坐标系中的最大偏离pitch、roll轴角度
    float plane_dist_threshold;      //地面方程参数估计时的距离阈值
    float groundCali_deg_threshold;  //估计地面偏离的角度大于阈值时，进行地面矫正
    int n_ground_estimate;           //进行地面方程参数估计的迭代次数
    /**** 引导人员跟随算法参数 ****/
    int n_reflectPoint_closest;      // 高反射率点的近邻点数量
    float voxel_leaf_size;           // 体素滤波的网格大小值
    float intensity_threshold;       // 雷达反射强度阈值
    float r_reflectPoint_search;     // 高反射率点的近邻搜索半径范围
    float std_target_size;           // 目标矩形贴纸的对角线长度
    /**** 栅格地图生成算法参数 ****/
    double max_x;          // 直通滤波的数值范围           
    double min_x;
    double max_y;
    double min_y;
    double max_z;
    double min_z;          // min_z是雷达离地面高度
    double grid_scale;     // 每个栅格边长表示的实际长度
    int n_gridmap_x;       // x方向栅格个数
    int n_gridmap_y;       // y方向栅格个数
    int n_pixel_per_grid;  // 每个栅格边长占据的像素个数，用于opencv显示
    /**** 主函数控制参数 ****/
    bool visualize;               // 是否进行可视化
    string pcapAddr;
    int pattern_select_switch;   //0是人员跟随，1是自主导航

    InitParams() :
        visualize_ground(false),
        r_min_square(0.3 * 0.3),
        r_max_square(20*20),
        n_bins(30),
        n_segments(180),
        max_dist_to_line(0.15),
        max_slope(1),
        n_threads(4),
        max_error_square(0.01),
        long_threshold(2.0),
        max_long_height(0.1),
        max_start_height(0.2),
        sensor_height(0.2),
        line_search_angle(0.2),
        max_deviation_deg(15),
        plane_dist_threshold(0.05),
        groundCali_deg_threshold(2),
        n_ground_estimate(40),
        voxel_leaf_size(0.035),
        intensity_threshold(196),
        r_reflectPoint_search(0.1),
        n_reflectPoint_closest(4),
        std_target_size(0.35355),
        max_x(11),                    //直通滤波的数值范围
        min_x(-3.0),
        max_y(6.0),
        min_y(-6.0),
        max_z(0.4),
        min_z(-1.5),                  
        n_gridmap_x(226),
        n_gridmap_y(226),
        grid_scale(0.1),
        n_pixel_per_grid(2),
        pcapAddr(""),
        visualize(false),
        pattern_select_switch(0) {}
};

#endif