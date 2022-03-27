//
// Created by wsh on 2021/8/26.
//

#include "AutoNavigation.h"

// TEST
#include "testFile/grid.h"
#include "testFile/test_map.h"

// LIDAR parameters
//string paramAdress = LIDAR_PARAM_PATH;
//string startIconAdrress = LIDAR_START_ICON_PATH;
//string endIconAdrress = CONFIG_END_ICON_PATH;

constexpr int localPort = 6918; // receive pose from robot {x y z roll pitch yaw}
constexpr int POSE_SE_UDP_LENGTH = 31; // head->PoseSE, {x y z roll pitch yaw}

AutoNavigation::AutoNavigation() : lcmMonitor("udpm://239.255.12.21:1221?ttl=255")
{
    cfg = new ConfigParameters(CONFIG_YAML_PATH);
    init();
}

AutoNavigation::AutoNavigation(ConfigParameters &cfg_) : lcmMonitor("udpm://239.255.12.21:1221?ttl=255")
{
    cfg = &cfg_;
    init();
}

AutoNavigation::AutoNavigation(const std::string &cfg_path_) : lcmMonitor("udpm://239.255.12.21:1221?ttl=255")
{
    cfg = new ConfigParameters(cfg_path_);
    init();
}

AutoNavigation::~AutoNavigation()
{
    if (!ifExternCfg)
    {
        delete cfg;
    }
}

void AutoNavigation::init()
{
    if (cfg->monitor_use_lcm)
    {
//        lcmMonitor = lcm::LCM("udpm://239.255.12.21:1221?ttl=255");
        if (!lcmMonitor.good())
        {
            std::cout << "[ ERROR ] -> LCM initialize failed!" << std::endl;
            exit(-1);
        }
    }

    ifExternCfg = false;
    _running = false;

    if (cfg->follow_task == Follow_task::route_follow)
    {
        if (cfg->no_block_map)
        {
            loop_planning = LoopFunction("route follow with no_block map", cfg->get_planning_period(), [this]
            { getPathCommandGivenRoute_noBlock(); });
        }
        else
        {
            loop_planning = LoopFunction("route follow", cfg->get_planning_period(), [this]
            { getPathCommandGivenRoute(); });
        }
    }
    else if (cfg->follow_task == Follow_task::target_follow)
    {
        loop_planning = LoopFunction("target follow", cfg->get_planning_period(), [this]
        { getPathCommand(); });
    }
    else if (cfg->follow_task == Follow_task::cin_points_follow)
    {
        loop_planning = LoopFunction("cin points follow", cfg->get_planning_period(), [this]
        { getPathCommandCin(); });
    }


    loop_remoteSend = LoopFunction("sendRemote", cfg->get_send_period(),
                                   boost::bind(&AutoNavigation::sendPathCommand, this));

    if (cfg->get_use_udp())
    {
        udp.init(cfg->get_ip_addr(), cfg->get_port(), 0, 0);
        strncpy((char *) sendBuf, "Auto", 5);
    }
    receiveLegOdmUdp.init("", 0, localPort, 0);

    /// route following
    loop_selfLocalizationWOLidar = LoopFunction("selfLocalizationWOLidar", cfg->self_localization_WO_lidar_period,
                                                [this]
                                                { selfLocalizationWOLidar(); });
    routePoints = cfg->route_points;
    routeTargetThreshold = cfg->route_distance_threshold;
    selfLocalizationQueue.push({0, 0, 0});
    numRoutePoints = routePoints.size();

    clock_gettime(CLOCK_MONOTONIC, &l_startTime);
}

//void AutoNavigation::getPathCommandTest()
//{
//    //TODO test
//    GridMapTest GridFileReadTest(TEST_MAP_PATH);
////    usleep(1000);
//    auto gridMap = GridFileReadTest.GetGridVectorTest();
//    std::cout << "row: " << gridMap.size() << std::endl << "col: " << gridMap[0].size() << std::endl;
//
//    path_planner.setGridMap(gridMap);
//    path_planner.setRobotSelfRadius(cfg->get_self_block_radius_grid());
//
//    int sourceX = 14; //TODO
//    int sourceY = 69; //TODO
//    int targetX = 51; //TODO
//    int targetY = 104; //TODO
//    int init_direcX = cfg->get_init_direction()[0];
//    int init_direcY = cfg->get_init_direction()[1];
//    int init_len = cfg->get_default_forward_distance_grid();
//    double Rd = cfg->get_default_curve_radius_grid();
//    auto path = path_planner.arc_planning(sourceX, sourceY, targetX, targetY, init_direcX, init_direcY, init_len, Rd);
//    // print path
//    AStar::print_path(path_planner.info2vec(path));
//
//    float resolution = cfg->get_map_grid_resolution();
//    float vel = cfg->get_default_vel();
//    float w = cfg->get_max_angular_vel();
//    auto cmd = AStar::path2cmd(path, resolution, vel, w);
//
//    //push the path command into queue.
//    CmdQueue.push(cmd);
//}

void AutoNavigation::getPathCommand()
{
    // TEST generate a map
    /*GridMapTest GridFileReadTest(TEST_MAP_PATH);
    auto gridMap = GridFileReadTest.GetGridVectorTest();
    LidarMap tmp;
    tmp.start_row = 14;
    tmp.start_column = 69;
    tmp.target_row = 51;
    tmp.target_column = 104;
    tmp.map = gridMap;
    newMapQueue.push(tmp);*/
//    vector<cv::Vec4f> tmp_line;
//    lineCoordQueue.push(tmp_line);
    /****************************************/
    clock_gettime(CLOCK_MONOTONIC, &startTime);
    newMapQueue.pop_uptodate(newMap);

    float resolution = cfg->get_map_grid_resolution();

    /*if (cfg->get_add_border_line())
    {
        lineCoordQueue.pop_anyway(&newline);
        addLinetoMap(newMap.map, resolution, newline, newMap.start_row, newMap.start_column, cfg->line_k_max);
    }*/

    path_planner.setGridMap(newMap.map);
    path_planner.setRobotSelfRadius(cfg->get_self_block_radius_grid());
    path_planner.setAstarCost(cfg->astar_cost_factors);

    int sourceX = newMap.start_row + cfg->get_lidar_pos_bias_grid()[0];
    int sourceY = newMap.start_column + cfg->get_lidar_pos_bias_grid()[1];

    int targetX = newMap.target_row; // TODO
    int targetY = newMap.target_column; // TODO

    int init_direcX = cfg->get_init_direction()[0];
    int init_direcY = cfg->get_init_direction()[1];
    int init_len = cfg->get_default_forward_distance_grid();
    double Rd = cfg->get_default_curve_radius_grid();
    double distance_threshold = cfg->get_target_distance_threshold_grid();
    auto path = path_planner.arc_planning(sourceX, sourceY, targetX, targetY, init_direcX, init_direcY, init_len, Rd,
                                          distance_threshold);

    // print path
//    AStar::print_path(path_planner.info2vec(path));

    float vel = cfg->get_default_vel();
    float w = cfg->get_max_angular_vel();
    auto cmd = AStar::path2cmd(path, resolution, vel, w);

    //push the path command into queue.
    CmdQueue.push(cmd);

//    clock_gettime(CLOCK_MONOTONIC, &endTime);
//    timePassed = (float) ((double) (endTime.tv_nsec - startTime.tv_nsec) / 1.e6 +
//                          (double) (endTime.tv_sec - startTime.tv_sec) * 1e3);
//    std::cout << "send_time: " << timePassed << " ms" << std::endl;


    /***************************************************************Send Image**************************************/
//    std::cout << cfg->show_map_image << '\t' << udp_sendImg.sendingImg() << endl;
    if (cfg->show_map_image && udp_sendImg.sendingImg())
    {
        sendMapImage(newMap.map, path);
    }
/*******************************************************************************************************************/
}

void AutoNavigation::getPathCommandCin()
{
    // TEST generate a map
    /*GridMapTest GridFileReadTest(TEST_MAP_PATH);
    auto gridMap = GridFileReadTest.GetGridVectorTest();
    LidarMap tmp;
    tmp.start_row = 14;
    tmp.start_column = 69;
    tmp.target_row = 51;
    tmp.target_column = 104;
    tmp.map = gridMap;
    newMapQueue.push(tmp);*/
//    vector<cv::Vec4f> tmp_line;
//    lineCoordQueue.push(tmp_line);
    /****************************************/
    float xxx, yyy;
    std::cout << "input target coordinate: " << std::endl;
    std::cin >> xxx >> yyy;
    clock_gettime(CLOCK_MONOTONIC, &startTime);
    newMapQueue.pop_uptodate(newMap);

    float resolution = cfg->get_map_grid_resolution();

    /*if (cfg->get_add_border_line())
    {
        lineCoordQueue.pop_anyway(&newline);
        addLinetoMap(newMap.map, resolution, newline, newMap.start_row, newMap.start_column, cfg->line_k_max);
    }*/

    path_planner.setGridMap(newMap.map);
    path_planner.setRobotSelfRadius(cfg->get_self_block_radius_grid());
    path_planner.setAstarCost(cfg->astar_cost_factors);

    int sourceX = newMap.start_row + cfg->get_lidar_pos_bias_grid()[0];
    int sourceY = newMap.start_column + cfg->get_lidar_pos_bias_grid()[1];

//    int targetX = newMap.target_row; // TODO
//    int targetY = newMap.target_column; // TODO

    int targetX, targetY;
    targetX = newMap.start_row + (int) (xxx / 0.1);
    targetY = newMap.start_column + (int) (yyy / 0.1);

    int init_direcX = cfg->get_init_direction()[0];
    int init_direcY = cfg->get_init_direction()[1];
    int init_len = cfg->get_default_forward_distance_grid();
    double Rd = cfg->get_default_curve_radius_grid();
    double distance_threshold = cfg->get_target_distance_threshold_grid();
    auto path = path_planner.arc_planning(sourceX, sourceY, targetX, targetY, init_direcX, init_direcY, init_len, Rd,
                                          distance_threshold);

    // print path
//    AStar::print_path(path_planner.info2vec(path));

    float vel = cfg->get_default_vel();
    float w = cfg->get_max_angular_vel();
    auto cmd = AStar::path2cmd(path, resolution, vel, w);

    //push the path command into queue.
    CmdQueue.push(cmd);

//    clock_gettime(CLOCK_MONOTONIC, &endTime);
//    timePassed = (float) ((double) (endTime.tv_nsec - startTime.tv_nsec) / 1.e6 +
//                          (double) (endTime.tv_sec - startTime.tv_sec) * 1e3);
//    std::cout << "send_time: " << timePassed << " ms" << std::endl;


    /***************************************************************Send Image**************************************/
//    std::cout << cfg->show_map_image << '\t' << udp_sendImg.sendingImg() << endl;
    if (cfg->show_map_image && udp_sendImg.sendingImg())
    {
        sendMapImage(newMap.map, path);
    }
/*******************************************************************************************************************/
}

void AutoNavigation::getPathCommandGivenRoute()
{
    clock_gettime(CLOCK_MONOTONIC, &startTime);
    newMapQueue.pop_uptodate(newMap);
    /*int rr = newMap.map.size();
    int cc = newMap.map[0].size();
    for (int iiii = 0; iiii < rr; iiii++)
    {
        for (int jjjj = 0; jjjj < cc; jjjj++)
        {
            newMap.map[iiii][jjjj] = 0;
        }
    }*/

    float resolution = cfg->get_map_grid_resolution();

    /*if (cfg->get_add_border_line())
    {
        lineCoordQueue.pop_anyway(&newline);
        addLinetoMap(newMap.map, resolution, newline, newMap.start_row, newMap.start_column, cfg->line_k_max);
    }*/

    path_planner.setGridMap(newMap.map);
    path_planner.setRobotSelfRadius(cfg->get_self_block_radius_grid());
    path_planner.setAstarCost(cfg->astar_cost_factors);

    int sourceX = newMap.start_row + cfg->get_lidar_pos_bias_grid()[0];
    int sourceY = newMap.start_column + cfg->get_lidar_pos_bias_grid()[1];

//    int targetX = newMap.target_row; // TODO
//    int targetY = newMap.target_column; // TODO

    float xCurrent, yCurrent, thetaCurrent;
    float xTargetRelBody = 0;
    float yTargetRelBody = 0;

    odometry_out_channel.pop_uptodate(newLidarOdometer);

    std::vector<float> tmp_location;
    tmp_location.resize(3);
    selfLocalizationQueue.pop_anyway(&tmp_location);

    if (cfg->monitor_use_lcm)
    {
        monitorOdometer.odomLidar[0] = newLidarOdometer.transformDataSum[3];
        monitorOdometer.odomLidar[1] = newLidarOdometer.transformDataSum[4];
        monitorOdometer.odomLidar[2] = newLidarOdometer.transformDataSum[2];
    }

    /// update 'xCurrent' 'yCurrent' 'thetaCurrent'
    if (cfg->self_localization_mode == Self_localization_mode::only_lidar)
    {
        xCurrent = newLidarOdometer.transformDataSum[3];
        yCurrent = newLidarOdometer.transformDataSum[4];
        thetaCurrent = newLidarOdometer.transformDataSum[2];
    }
    else if ((cfg->self_localization_mode == Self_localization_mode::only_SE) ||
             (cfg->self_localization_mode == Self_localization_mode::xy_SE_theta_HWT))
    {
        xCurrent = tmp_location[0];
        yCurrent = tmp_location[1];
        thetaCurrent = tmp_location[2];
    }
    else
    {
        if (cfg->self_localization_mode == Self_localization_mode::replace_lidarXY_SE)
        {
            xCurrent = tmp_location[0];
            yCurrent = tmp_location[1];
            thetaCurrent = newLidarOdometer.transformDataSum[2];
        }
        else
        {
            xCurrent = newLidarOdometer.transformDataSum[3];
            yCurrent = newLidarOdometer.transformDataSum[4];
            thetaCurrent = tmp_location[2];
        }
    }

    /// calculate 'xTargetRelBody' 'yTargetRelBody'
    calTargetGivenRoute(xCurrent, yCurrent, thetaCurrent, xTargetRelBody, yTargetRelBody);

    if (cfg->monitor_use_lcm)
    {
        monitorOdometer.odomCurrent[0] = xCurrent;
        monitorOdometer.odomCurrent[1] = yCurrent;
        monitorOdometer.odomCurrent[2] = thetaCurrent;
    }

    /// calculate target in gridmap
    int targetX, targetY;
    targetX = newMap.start_row - (int) (yTargetRelBody / resolution);
    targetY = newMap.start_column + (int) (xTargetRelBody / resolution);

    int init_direcX = cfg->get_init_direction()[0];
    int init_direcY = cfg->get_init_direction()[1];
    int init_len = cfg->get_default_forward_distance_grid();
    double Rd = cfg->get_default_curve_radius_grid();
    double distance_threshold = cfg->get_target_distance_threshold_grid();
    auto path = path_planner.arc_planning(sourceX, sourceY, targetX, targetY, init_direcX, init_direcY, init_len, Rd,
                                          distance_threshold);

    // print path
//    AStar::print_path(path_planner.info2vec(path));

    float vel = cfg->get_default_vel();
    float w = cfg->get_max_angular_vel();
    auto cmd = AStar::path2cmd(path, resolution, vel, w);

    //push the path command into queue.
    CmdQueue.push(cmd);

    // lcm publish
    if (cfg->monitor_use_lcm)
        lcmMonitor.publish("monitorOdometer", &monitorOdometer);

//    clock_gettime(CLOCK_MONOTONIC, &endTime);
//    timePassed = (float) ((double) (endTime.tv_nsec - startTime.tv_nsec) / 1.e6 +
//                          (double) (endTime.tv_sec - startTime.tv_sec) * 1e3);
//    std::cout << "send_time: " << timePassed << " ms" << std::endl;


    /***************************************************************Send Image**************************************/
//    std::cout << cfg->show_map_image << '\t' << udp_sendImg.sendingImg() << endl;
    if (cfg->show_map_image && udp_sendImg.sendingImg())
    {
        sendMapImage(newMap.map, path);
    }
/*******************************************************************************************************************/
}

void AutoNavigation::getPathCommandGivenRoute_noBlock()
{
    clock_gettime(CLOCK_MONOTONIC, &startTime);

    float resolution = cfg->get_map_grid_resolution();

    /*if (cfg->get_add_border_line())
    {
        lineCoordQueue.pop_anyway(&newline);
        addLinetoMap(newMap.map, resolution, newline, newMap.start_row, newMap.start_column, cfg->line_k_max);
    }*/

    auto tmp_map = std::vector<std::vector<int>>(240, std::vector<int>(240, 0));
//    tmp_map[0][0] = 1;
    auto tmp_start_row = 120;
    auto tmp_start_col = 120;

    path_planner.setGridMap(tmp_map);
    path_planner.setRobotSelfRadius(cfg->get_self_block_radius_grid());
    path_planner.setAstarCost(cfg->astar_cost_factors);

    int sourceX = tmp_start_row + cfg->get_lidar_pos_bias_grid()[0];
    int sourceY = tmp_start_col + cfg->get_lidar_pos_bias_grid()[1];

//    int targetX = newMap.target_row; // TODO
//    int targetY = newMap.target_column; // TODO

    float xCurrent, yCurrent, thetaCurrent;
    float xTargetRelBody = 0;
    float yTargetRelBody = 0;

    /// update 'xCurrent' 'yCurrent' 'thetaCurrent'
    if (cfg->self_localization_mode == Self_localization_mode::only_lidar)
    {
        odometry_out_channel.pop_uptodate(newLidarOdometer);

        xCurrent = newLidarOdometer.transformDataSum[3];
        yCurrent = newLidarOdometer.transformDataSum[4];
        thetaCurrent = newLidarOdometer.transformDataSum[2];
    }
    else if ((cfg->self_localization_mode == Self_localization_mode::only_SE) ||
             (cfg->self_localization_mode == Self_localization_mode::xy_SE_theta_HWT))
    {
        std::vector<float> tmp_location;
        tmp_location.resize(3);
        selfLocalizationQueue.pop_anyway(&tmp_location);

        xCurrent = tmp_location[0];
        yCurrent = tmp_location[1];
        thetaCurrent = tmp_location[2];
    }
    else
    {
        odometry_out_channel.pop_uptodate(newLidarOdometer);

        std::vector<float> tmp_location;
        tmp_location.resize(3);
        selfLocalizationQueue.pop_anyway(&tmp_location);

        if (cfg->self_localization_mode == Self_localization_mode::replace_lidarXY_SE)
        {
            xCurrent = tmp_location[0];
            yCurrent = tmp_location[1];
            thetaCurrent = newLidarOdometer.transformDataSum[2];
        }
        else
        {
            xCurrent = newLidarOdometer.transformDataSum[3];
            yCurrent = newLidarOdometer.transformDataSum[4];
            thetaCurrent = tmp_location[2];
        }
    }

    /// calculate 'xTargetRelBody' 'yTargetRelBody'
    calTargetGivenRoute(xCurrent, yCurrent, thetaCurrent, xTargetRelBody, yTargetRelBody);

    if (cfg->monitor_use_lcm)
    {
        monitorOdometer.odomLidar[0] = newLidarOdometer.transformDataSum[3];
        monitorOdometer.odomLidar[1] = newLidarOdometer.transformDataSum[4];
        monitorOdometer.odomLidar[2] = newLidarOdometer.transformDataSum[2];

        monitorOdometer.odomCurrent[0] = xCurrent;
        monitorOdometer.odomCurrent[1] = yCurrent;
        monitorOdometer.odomCurrent[2] = thetaCurrent;
    }

    /// calculate target in gridmap
    int targetX, targetY;
    targetX = tmp_start_row - (int) (yTargetRelBody / resolution);
    targetY = tmp_start_col + (int) (xTargetRelBody / resolution);

    int init_direcX = cfg->get_init_direction()[0];
    int init_direcY = cfg->get_init_direction()[1];
    int init_len = cfg->get_default_forward_distance_grid();
    double Rd = cfg->get_default_curve_radius_grid();
    double distance_threshold = cfg->get_target_distance_threshold_grid();

    AStar::Vec2d source = {sourceX, sourceY};
    AStar::Vec2d target = {targetX, targetY};
    AStar::Vec2d init_direction = {init_direcX, init_direcY};

    std::vector<AStar::Vec2d> simp_path;
    if ((xTargetRelBody != 0) || (yTargetRelBody != 0))
    {
        simp_path.push_back(source);
        simp_path.push_back(source + init_direction);
        simp_path.push_back(target);
    }
    else
        simp_path.push_back(source);
    auto smooth_path = path_planner.ArcSmooth(simp_path, Rd);

//    auto path = path_planner.arc_planning(sourceX, sourceY, targetX, targetY, init_direcX, init_direcY, init_len, Rd,
//                                          distance_threshold);

    // print path
//    AStar::print_path(path_planner.info2vec(path));

    float vel = cfg->get_default_vel();
    float w = cfg->get_max_angular_vel();
    auto cmd = AStar::path2cmd(smooth_path, resolution, vel, w);

    //push the path command into queue.
    CmdQueue.push(cmd);

    // lcm publish
    if (cfg->monitor_use_lcm)
        lcmMonitor.publish("monitorOdometer", &monitorOdometer);

//    clock_gettime(CLOCK_MONOTONIC, &endTime);
//    timePassed = (float) ((double) (endTime.tv_nsec - startTime.tv_nsec) / 1.e6 +
//                          (double) (endTime.tv_sec - startTime.tv_sec) * 1e3);
//    std::cout << "send_time: " << timePassed << " ms" << std::endl;


    /***************************************************************Send Image**************************************/
//    std::cout << cfg->show_map_image << '\t' << udp_sendImg.sendingImg() << endl;
    if (cfg->show_map_image && udp_sendImg.sendingImg())
    {
        sendMapImage(tmp_map, smooth_path);
    }
/*******************************************************************************************************************/
}

void AutoNavigation::sendMapImage(const std::vector<std::vector<int>> &map_, const std::vector<AStar::arcInfo> &path_)
{
    /************** Send new map info ***************/
    std::vector<uint8_t> newMapInfoBytes;
    uint8_t *tmp;
    uint32_t height = map_.size();
    uint32_t width = map_[0].size();
    // head
    newMapInfoBytes.push_back(0x12);
    newMapInfoBytes.push_back(0x34);
    // height bytes
    tmp = (uint8_t *) (&height);
    for (int i = 0; i < 4; i++)
    {
        newMapInfoBytes.push_back(tmp[i]);
    }
    // width bytes
    tmp = (uint8_t *) (&width);
    for (int i = 0; i < 4; i++)
    {
        newMapInfoBytes.push_back(tmp[i]);
    }
//    std::cout << "height: " << (int)height << "  width: " << (int)width << std::endl;
    udp_sendImg.SendBack(newMapInfoBytes.data(), 10);

    /************** Send path bytes ***************/
    std::vector<uint8_t> pathBytes;
    auto path_bytes_len = ConvertArcPath2Bytes(path_, pathBytes);
    if (path_bytes_len)
        udp_sendImg.SendBack(pathBytes.data(), path_bytes_len);

    /************** Send map bytes ***************/
    std::vector<uint8_t> tmpMap;
//    std::vector<uint8_t> tmpMapRow;
//    tmpMapRow.resize(width);
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            tmpMap.push_back(static_cast<uint8_t>(map_[i][j]));
//            tmpMapRow[j] = (static_cast<uint8_t>(newMap.map[i][j]));
        }
//        udp_sendImg.Send(tmpMapRow.data(), (int)width);
    }

    // heartbeat signal
    count_sendImg++;
    if (!std::fmod(count_sendImg, 1e8))
        count_sendImg = 0;
    if (!std::fmod(count_sendImg, 10))
        tmpMap[0] = 0;

    udp_sendImg.SendBack(tmpMap.data(), tmpMap.size());
//        usleep(18000);
//    uint8_t end = 0xf1;
//    udp_sendImg.Send(&end, 1);
    clock_gettime(CLOCK_MONOTONIC, &endTime);
    timePassed = (float) ((double) (endTime.tv_nsec - startTime.tv_nsec) / 1.e6 +
                          (double) (endTime.tv_sec - startTime.tv_sec) * 1e3);
//    std::cout << "send_time: " << timePassed << " ms" << std::endl;
}

void AutoNavigation::selfLocalizationWOLidar()
{
    if ((cfg->self_localization_mode == Self_localization_mode::only_SE) ||
        (cfg->self_localization_mode == Self_localization_mode::replace_lidarXY_SE) ||
        (cfg->self_localization_mode == Self_localization_mode::replace_lidarTheta_SE) ||
        (cfg->self_localization_mode == Self_localization_mode::xy_SE_theta_HWT))
    {
        float xBody, yBody, thetaBody;

        /// receive body position XY from NUC.
        uint8_t recvBuf[POSE_SE_UDP_LENGTH];
        int len;
        for (unsigned char &elem: recvBuf)
            elem = 0;
        if (cfg->get_use_udp())
        {
            len = receiveLegOdmUdp.Receive(recvBuf);
            if (len == POSE_SE_UDP_LENGTH)
            {
                auto head = std::string(recvBuf, recvBuf + 6);
                if (head == "PoseSE")
                {
                    xBody = SocketTools::ConvertByte2Float((unsigned char *) (recvBuf + 7));
                    yBody = SocketTools::ConvertByte2Float((unsigned char *) (recvBuf + 11));
                    thetaBody = SocketTools::ConvertByte2Float((unsigned char *) (recvBuf + 27));

//                    std::cout << "xBody: " << xBody << '\t' <<
//                              "yBody" << yBody << '\t' <<
//                              "thetaBody" << thetaBody << std::endl;

                    if (cfg->monitor_use_lcm)
                    {
                        monitorOdometer.odomSE[0] = xBody;
                        monitorOdometer.odomSE[1] = yBody;
                        monitorOdometer.odomSE[2] = thetaBody;
                    }

                    if (cfg->self_localization_mode == Self_localization_mode::only_SE)
                    {
                        xReplace = xBody;
                        yReplace = yBody;
                        thetaReplace = thetaBody;
                    }
                    else if ((cfg->self_localization_mode == Self_localization_mode::replace_lidarXY_SE) ||
                             (cfg->self_localization_mode == Self_localization_mode::xy_SE_theta_HWT))
                    {
                        xReplace = xBody;
                        yReplace = yBody;
                    }
                    else
                    { thetaReplace = thetaBody; }
                }
                else
                {
                    std::cout << "[PoseSE] Head received error!" << std::endl;
                }
            }
            else
            {
                std::cout << "[PoseSE] Length received error!" << " (" << len << ")" << std::endl;
                return;
            }
        }
    }

    if ((cfg->self_localization_mode == Self_localization_mode::replace_lidarTheta_HWT) ||
        (cfg->self_localization_mode == Self_localization_mode::xy_SE_theta_HWT))
    {
        /// read Z-axis euler angle.
        float tmpThetaHWT = readThetaFromHWT101.readZ_rad();
        // TODO read error
        if (tmpThetaHWT != -12345)
        {
            thetaReplace = tmpThetaHWT;
            if (cfg->monitor_use_lcm)
                monitorOdometer.thetaGyro = tmpThetaHWT;
        }
    }

    selfLocalizationQueue.push({xReplace, yReplace, thetaReplace});

    clock_gettime(CLOCK_MONOTONIC, &l_endTime);
    timePassed = (float) ((double) (l_endTime.tv_nsec - l_startTime.tv_nsec) / 1.e6 +
                          (double) (l_endTime.tv_sec - l_startTime.tv_sec) * 1e3);
//    std::cout << "localize_time: " << timePassed << " ms" << std::endl;
    clock_gettime(CLOCK_MONOTONIC, &l_startTime);
}

void AutoNavigation::calTargetGivenRoute(float xBody_, float yBody_, float theta_, float &xTarget, float &yTarget)
{
    if (numRoutePoints == 0)
    {
        xTarget = 0;
        yTarget = 0;
        return;
    }
    else
    {
        while (currentRoutePointIndex < (numRoutePoints - 1)) // not the last one.
        {
            float xDelta = routePoints[currentRoutePointIndex][0] - xBody_;
            float yDelta = routePoints[currentRoutePointIndex][1] - yBody_;
            if (sqrt(pow(xDelta, 2) + pow(yDelta, 2)) > routeTargetThreshold)
            {
                xTarget = cos(theta_) * xDelta + sin(theta_) * yDelta;
                yTarget = -sin(theta_) * xDelta + cos(theta_) * yDelta;
//                std::cout << "xTmp: " << xTarget << std::endl;
//                std::cout << "yTmp: " << yTarget << std::endl;
                if (cfg->monitor_use_lcm)
                {
                    monitorOdometer.currentWayPoint[0] = xTarget;
                    monitorOdometer.currentWayPoint[1] = yTarget;
                }
                return;
            }
            else
            {
                currentRoutePointIndex++;
                continue;
            }
        }

        // the last one
        float xDelta = routePoints[currentRoutePointIndex][0] - xBody_;
        float yDelta = routePoints[currentRoutePointIndex][1] - yBody_;
        if (sqrt(pow(xDelta, 2) + pow(yDelta, 2)) > cfg->get_target_distance_threshold())
        {
            xTarget = cos(theta_) * xDelta + sin(theta_) * yDelta;
            yTarget = -sin(theta_) * xDelta + cos(theta_) * yDelta;
        }
        else
        {
            xTarget = 0;
            yTarget = 0;
        }

        if (cfg->monitor_use_lcm)
        {
            monitorOdometer.currentWayPoint[0] = xTarget;
            monitorOdometer.currentWayPoint[1] = yTarget;
        }

        return;
    }
}

void AutoNavigation::sendPathCommand()
{
    if (CmdQueue.empty())
    {
        // if the command Queue is empty, wait for a while
        CmdQueue.pop_wait(&pathCmd);

        // reset start time
        clock_gettime(CLOCK_MONOTONIC, &startTime);

        // zero the curve count
        currentRow = 0;

        // update the total curve count -> Row of 2-d Vector.
        totalCurveCount = pathCmd.size();

        // update the totalSegTime to the time of fist curve row.
        totalSegTime = (float) (pathCmd[0][0] * 1000.0); // ms

        isFirstCmd = true;
    }
    else if (CmdQueue.isUPdate())
    {
        // if the command is update, get the new command.
        CmdQueue.pop_anyway(&pathCmd);

        // reset start time
        clock_gettime(CLOCK_MONOTONIC, &startTime);

        // zero the line count
        currentRow = 0;

        // update the total curve count -> Row of 2-d Vector.
        totalCurveCount = pathCmd.size();

        // update the totalSegTime to the time of fist curve row.
        totalSegTime = (float) (pathCmd[0][0] * 1000.0); // ms

        isFirstCmd = true;
    }
    else
    {
        isFirstCmd = false;
    }

    while (currentRow < totalCurveCount)
    {
        // update end time
        clock_gettime(CLOCK_MONOTONIC, &endTime);

        // calculate the time spent in one segment. (ms)
        timePassed = (float) ((double) (endTime.tv_nsec - startTime.tv_nsec) / 1.e6 +
                              (double) (endTime.tv_sec - startTime.tv_sec) * 1e3);

        if (timePassed < totalSegTime)
        {
            udpSend(isFirstCmd, pathCmd[currentRow][1], pathCmd[currentRow][2], pathCmd[currentRow][3]);

            return; // ready to next loop.
        }
        else
        {
            // reset start time
            clock_gettime(CLOCK_MONOTONIC, &startTime);

            // the next curve
            currentRow++;

            if (currentRow < totalCurveCount)
            {
                // update the totalSegTime
                totalSegTime = (float) (pathCmd[currentRow][0] * 1000.0); // ms
            }
            continue;
        }
    }

    // all calculated commands have been sent. Over.
    udpSend(false, 0, 0, 0);
}

void AutoNavigation::sendPathCommand_Count()
{
    if (CmdQueue.empty())
    {
        // if the command Queue is empty, wait for a while
        CmdQueue.pop_wait(&pathCmd);

        // reset start time
//        clock_gettime(CLOCK_MONOTONIC, &startTime);

        // zero the curve count
        currentRow = 0;
        sendCount_inSeg = 0;

        // update the total curve count -> Row of 2-d Vector.
        totalCurveCount = pathCmd.size();

        // update the totalSegTime to the time of fist curve row.
//        totalSegTime = (float) (pathCmd[0][0] * 1000.0); // ms
        currentSegCount = (int) (pathCmd[0][0] / cfg->get_send_period());

        isFirstCmd = true;
    }
    else if (CmdQueue.isUPdate())
    {
        // if the command is update, get the new command.
        CmdQueue.pop_anyway(&pathCmd);

        // reset start time
//        clock_gettime(CLOCK_MONOTONIC, &startTime);

        // zero the line count
        currentRow = 0;
        sendCount_inSeg = 0;

        // update the total curve count -> Row of 2-d Vector.
        totalCurveCount = pathCmd.size();

        // update the totalSegTime to the time of fist curve row.
//        totalSegTime = (float) (pathCmd[0][0] * 1000.0); // ms
        currentSegCount = (int) (pathCmd[0][0] / cfg->get_send_period());

        isFirstCmd = true;
    }
    else
    {
        isFirstCmd = false;
    }

    while (currentRow < totalCurveCount)
    {
        // update end time
//        clock_gettime(CLOCK_MONOTONIC, &endTime);

        // calculate the time spent in one segment. (ms)
//        timePassed = (float) ((double) (endTime.tv_nsec - startTime.tv_nsec) / 1.e6 +
//                              (double) (endTime.tv_sec - startTime.tv_sec) * 1e3);

//        if (timePassed < totalSegTime)
        if (sendCount_inSeg < currentSegCount)
        {
            udpSend(isFirstCmd, pathCmd[currentRow][1], pathCmd[currentRow][2], pathCmd[currentRow][3]);
            ++sendCount_inSeg;
            return; // ready to next loop.
        }
        else
        {
            // reset start time
//            clock_gettime(CLOCK_MONOTONIC, &startTime);

            // the next curve
            currentRow++;

            sendCount_inSeg = 0;

            if (currentRow < totalCurveCount)
            {
                // update the totalSegTime
//                totalSegTime = (float) (pathCmd[currentRow][0] * 1000.0); // ms
                currentSegCount = (int) (pathCmd[currentRow][0] / cfg->get_send_period());
            }
            continue;
        }
    }

    // all calculated commands have been sent. Over.
    udpSend(false, 0, 0, 0);
}

void AutoNavigation::start()
{
    if (_running)
    {
        std::cout << "[Task-AutoNavigation] The task has already been running!" << std::endl;
        return;
    }
    _running = true;

    /*// start lidar task
    IniParam Param;
    if (!getParameter(paramAdress, Param))
    {
        cout << "read paramter file error!" << endl;
    }
    th_lidar = std::thread(MutLidar, ref(Param), ref(newMapQueue), ref(_running), "");
    std::cout << "[Task-Lidar] Start running!" << std::endl;*/

    if (cfg->follow_task == Follow_task::target_follow)
    {
        // start single lidar target-follow task
        getParameter(setParam);
        th_lidar = std::thread(LidarMapThreadFun, ref(_running), ref(setParam), ref(newMapQueue));
        std::cout << "[Task-Lidar] Start running target-follow task!" << std::endl;
    }
    else if (cfg->follow_task == Follow_task::route_follow)
    {
        // start single lidar route-follow task
        getParameter(setParam);
        th_lidar = std::thread(AutonomousNav, ref(_running), ref(setParam), ref(newMapQueue), ref(odometry_out_channel),
                               ref(featureExtra_out_channel));
        std::cout << "[Task-Lidar] Start running route-follow task!" << std::endl;
    }
    else if (cfg->follow_task == Follow_task::cin_points_follow)
    {
        // start single lidar cin_points-follow task
        getParameter(setParam);
        th_lidar = std::thread(AutonomousNav, ref(_running), ref(setParam), ref(newMapQueue), ref(odometry_out_channel),
                               ref(featureExtra_out_channel));
        std::cout << "[Task-Lidar] Start running cin_points-follow task!" << std::endl;
    }


    /*if (cfg->get_add_border_line())
    {
        // start line detect task
        th_line = std::thread(lans_detect, ref(linePointQueue), ref(lineCoordQueue), ref(_running), ref(cfg->show_lines));
        std::cout << "[Task-Line_detection] Start running!" << std::endl;
    }*/


    // start planning the path and generate control command
    loop_planning.start();

    // start sending command to remote
    loop_remoteSend.start();

    // start get the update target. receive body position XY from NUC. receive Z-axis euler angle from sensor. calculate the point's relative coordinate to body.
    if ((cfg->follow_task == Follow_task::route_follow) &&
        (cfg->self_localization_mode != Self_localization_mode::only_lidar))
    {
        if ((cfg->self_localization_mode == Self_localization_mode::replace_lidarTheta_HWT) ||
            (cfg->self_localization_mode == Self_localization_mode::xy_SE_theta_HWT))
        { readThetaFromHWT101.setup(); }
        loop_selfLocalizationWOLidar.start();
    }

    std::cout << "ALL TASKS STARTED!" << std::endl;
}

void AutoNavigation::stop()
{
    if (!_running)
    {
        std::cout << "[Task-AutoNavigation] The task has already been stopped!" << std::endl;
        return;
    }
    _running = false;

    // stop sending command to remote
    loop_remoteSend.stop();

    // stop planning the path and generate control command
    loop_planning.stop();

    // stop the lidar task
    std::cout << "[Task-Lidar] Stopping!" << std::endl;
    th_lidar.join();
    std::cout << "[Task-Lidar] Stopped!" << std::endl;

    /*if (cfg->get_add_border_line())
    {
        // stop the line detect task
        std::cout << "[Task-Line_detection] Stopping!" << std::endl;
        th_line.join();
        std::cout << "[Task-Line_detection] Stopped!" << std::endl;
    }*/

    std::cout << "ALL TASKS STOPPED!" << std::endl;
}

void AutoNavigation::udpSend(bool isFirstCmd_, float velX_, float velY_, float turnRate_)
{
    sendBuf[5] = isFirstCmd_ ? 0x01 : 0x00;

    auto tmp = (uint8_t *) (&velX_);
    sendBuf[6] = tmp[0];
    sendBuf[7] = tmp[1];
    sendBuf[8] = tmp[2];
    sendBuf[9] = tmp[3];

    tmp = (uint8_t *) (&velY_);
    sendBuf[10] = tmp[0];
    sendBuf[11] = tmp[1];
    sendBuf[12] = tmp[2];
    sendBuf[13] = tmp[3];

    tmp = (uint8_t *) (&turnRate_);
    sendBuf[14] = tmp[0];
    sendBuf[15] = tmp[1];
    sendBuf[16] = tmp[2];
    sendBuf[17] = tmp[3];

    if (cfg->get_use_udp())
    {
        udp.Send(sendBuf, AUTO_COMMAND_LENGTH);
        std::cout << "velX: " << velX_ << "\tvelY: " << velY_ << "\tturnRate: " << turnRate_ << std::endl;
    }
}

int AutoNavigation::ConvertArcPath2Bytes(const std::vector<AStar::arcInfo> &path_, std::vector<uint8_t> &bytes)
{
    std::vector<uint8_t> n;
    bytes.swap(n);

    if (path_.empty())
        return 0;

    int count = path_.size();
    auto arcPath = AStar::Arc::info2vec(path_);
    uint8_t *tmp;

    for (const auto &path: arcPath)
    {
        for (const auto &elem: path)
        {
            auto t = static_cast<float>(elem);
            tmp = (uint8_t *) (&t);
            for (int i = 0; i < 4; i++)
            {
                bytes.push_back(tmp[i]);
            }
        }
    }

    return bytes.size();
}

/*void
AutoNavigation::addLinetoMap(std::vector<std::vector<int>> &map_, float resolution, const vector<cv::Vec4f> &lines_,
                             int origin_x, int origin_y, float k_max)
{
    auto n_lines = lines_.size();
    if (lines_.empty() || (n_lines > 2))
        return;

    auto n_rows = map_.size();
    auto n_cols = map_[0].size();

    Eigen::ArrayXXf rowId(n_rows, n_cols);
    Eigen::ArrayXXf colId(n_rows, n_cols);

    for (int i = 0; i < n_rows; i++)
    {
        rowId.row(i) = Eigen::ArrayXXf::Constant(1, n_cols, i);
    }

    for (int i = 0; i < n_cols; i++)
    {
        colId.col(i) = Eigen::ArrayXf::Constant(n_rows, i);
    }

    for (int i_line = 0; i_line < n_lines; i_line++)
    {
        auto delta_x = lines_[i_line][0];
        auto delta_y = lines_[i_line][1];
        auto x0 = lines_[i_line][2] * resolution;
        auto y0 = lines_[i_line][3] * resolution;

        float k = delta_y / (delta_x + 1e-7);
        auto x0_grid = x0 + origin_x;
        auto y0_grid = y0 + origin_y;

        if (abs(k) > k_max) // Parallel to the robot's forward direction.
        {
            if ((x0_grid < 0) || (x0_grid > n_rows) || (x0 == 0))
                return;
            else if (x0 < 0)
            {
                // no more left
                for (int i = 0; i < (int) x0_grid + 1; i++)
                {
                    for (int j = 0; j < n_cols; j++)
                    {
                        map_[i][j] = 1;
                    }
                }
                return;
            }
            else
            {
                // no more right
                for (int i = (int) x0_grid; i < n_rows; i++)
                {
                    for (int j = 0; j < n_cols; j++)
                    {
                        map_[i][j] = 1;
                    }
                }
                return;
            }
        }
        else if (abs(k) < (1.0 / k_max)) // Perpendicular to the robot's forward direction.
        {
            if ((y0_grid < 0) || (y0_grid > n_cols) || (y0 == 0))
                return;
            else if (y0 > 0)
            {
                // no more front
                for (int j = (int) y0_grid; j < n_cols; j++)
                {
                    for (int i = 0; i < n_rows; i++)
                    {
                        map_[i][j] = 1;
                    }
                }
                return;
            }
            else
            {
                // no more back
                for (int j = 0; j < (int) y0_grid + 1; j++)
                {
                    for (int i = 0; i < n_rows; i++)
                    {
                        map_[i][j] = 1;
                    }
                }
                return;
            }
        }
        else
        {
            bool origin_pos = (0 >= (k * (0 - x0) + y0)); // True if the origin is above the line.
            Eigen::ArrayXXf relative_pos =
                    colId - (k * (rowId - x0_grid) + y0_grid); // True if the point is above the line.
            if (!origin_pos)
            {
                // block up the line
                for (int i = 0; i < n_rows; i++)
                {
                    for (int j = 0; j < n_cols; j++)
                    {
                        if (relative_pos(i, j) >= 0)
                            map_[i][j] = 1;
                    }
                }
            }
            else
            {
                // block below the line
                for (int i = 0; i < n_rows; i++)
                {
                    for (int j = 0; j < n_cols; j++)
                    {
                        if (relative_pos(i, j) <= 0)
                            map_[i][j] = 1;
                    }
                }
            }
        }
    }
}*/
