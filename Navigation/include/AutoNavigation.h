//
// Created by wsh on 2021/8/26.
//

#ifndef AUTONOMOUS_NAVIGATION_AUTONAVIGATION_H
#define AUTONOMOUS_NAVIGATION_AUTONAVIGATION_H

#define AUTO_COMMAND_LENGTH 18

#include <unistd.h>
#include <array>
#include "ConfigParameters.hpp"
#include "LockCircleQueue.hpp"
#include "AStar.hpp"
//#include "LidarMap_t.h"
//#include "MutiLidar.h"
#include "singleLidarShare.h"
//#include "ZED_whiteline.h"
//#include <opencv2/opencv.hpp>
#include "Loop.hpp"
#include "config_yaml.h"
#include "UDP.hpp"
#include "RecvRequest.hpp"

#include "ReadEulerZ_HWT101.hpp"

#include <lcm/lcm-cpp.hpp>
#include "lcm_monitor_odometer.hpp"


class AutoNavigation
{
public:
    AutoNavigation();

    explicit AutoNavigation(ConfigParameters &cfg_);

    explicit AutoNavigation(const std::string &cfg_path_);

    AutoNavigation(const AutoNavigation &) = delete;

    AutoNavigation &operator=(const AutoNavigation &) = delete;

    ~AutoNavigation();

    void init();

    void getPathCommand();

    void getPathCommandCin();

    void getPathCommandGivenRoute();

    void getPathCommandGivenRoute_noBlock();

    void calTargetGivenRoute(float xBody_, float yBody_, float theta_, float &xTarget, float &yTarget);

//    void getPathCommandTest();

    void sendPathCommand();

    void sendPathCommand_Count();

    void start();

    void stop();

    void udpSend(bool isFirstCmd_, float velX_, float velY_, float turnRate_);

    void manage();

//    static void addLinetoMap(std::vector<std::vector<int>> &map_, float resolution, const vector<cv::Vec4f> &lines_, int origin_x, int origin_y, float k_max);

    static int ConvertArcPath2Bytes(const std::vector<AStar::arcInfo> &path, std::vector<uint8_t> &bytes);

    void selfLocalizationWOLidar();

    void monitorOdometerData();

    void sendMapImage(const std::vector<std::vector<int>> &map_, const std::vector<AStar::arcInfo> &path_,
                      const std::array<float, 3> &trans = {0.0, 0.0, 0.0});

private:
    ConfigParameters *cfg;
    bool ifExternCfg;

    volatile bool _running;

    // threads
    std::thread th_lidar;
//    std::thread th_line;

    // loop thread
    LoopFunction loop_planning;
    LoopFunction loop_remoteSend;

    // route following
    LoopFunction loop_selfLocalizationWOLidar;
    LockCircleQueue<std::vector<float>, 1> selfLocalizationQueue; // relative to the move body frame.
    std::vector<std::vector<float>> routePoints; // relative to the initial world frame.
    float routeTargetThreshold;
    unsigned long numRoutePoints;
    int currentRoutePointIndex = 0;

    float xReplace = 0;
    float yReplace = 0;
    float thetaReplace = 0;

    // functional objects
    AStar::ArcPlanning path_planner;

    // Data sharing Container
    LidarMap_t newMapQueue;
//    LockCircleQueue<vector<vector<cv::Point2f>>, 1> linePointQueue;
//    LockCircleQueue<vector<cv::Vec4f>, 1> lineCoordQueue;
    LockCircleQueue<std::vector<std::vector<float>>, 1> CmdQueue;

    // the Map struct
    LidarMap newMap;

    // the current lines
//    std::vector<cv::Vec4f> newline;

    // Control command
    std::vector<std::vector<float>> pathCmd;
    int currentRow = 0;
    float timePassed = 0; // ms
    timespec startTime;
    timespec endTime;
    size_t totalCurveCount = 0;
    float totalSegTime = 0; // ms
    int currentSegCount = 0;
    int sendCount_inSeg = 0;
    bool isFirstCmd = false;
    float cmdX; // m/s
    float cmdY; // m/s
    float cmdW; // rad/s

    // UDP Client
    Udp udp;
    RecvRequest udp_sendImg;
    Udp receiveLegOdmUdp;
//    Udp udpTest;
    uint8_t sendBuf[AUTO_COMMAND_LENGTH];

    int count_sendImg = 0;

    ReadEulerZ_HWT101 readThetaFromHWT101;

    /// self-localization using lidar
    // ???????????????????????????
    Channel<OdometryOut> odometry_out_channel;// true???????????????
    // ??????????????????
    Channel<ExtractionOut> featureExtra_out_channel;// true???????????????
    InitParams setParam;// ??????????????????????????????
    OdometryOut newLidarOdometer;

    lcm::LCM lcmMonitor;
    lcm_monitor_odometer monitorOdometer{};

    timespec l_startTime;
    timespec l_endTime;

    LockCircleQueue<std::vector<std::vector<float>>, 1> pickedPointsList;
};

#endif //AUTONOMOUS_NAVIGATION_AUTONAVIGATION_H
