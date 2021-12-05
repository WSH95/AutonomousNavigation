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

constexpr int localPort = 0;

AutoNavigation::AutoNavigation()
{
    cfg = new ConfigParameters(CONFIG_YAML_PATH);
    ifExternCfg = false;
    _running = false;
    loop_planning = LoopFunction("planning", 0.1, [this]
    { getPathCommand(); });
    loop_remoteSend = LoopFunction("sendRemote", cfg->get_send_period(),
                                   boost::bind(&AutoNavigation::sendPathCommand, this));

    udp.init(cfg->get_ip_addr(), cfg->get_port(), localPort, 0);
    strncpy((char *) sendBuf, "Auto", 5);
}

AutoNavigation::AutoNavigation(ConfigParameters &cfg_)
{
    cfg = &cfg_;
    ifExternCfg = true;
    _running = false;
    loop_planning = LoopFunction("planning", 0.1, [this]
    { getPathCommand(); });
    loop_remoteSend = LoopFunction("sendRemote", cfg->get_send_period(),
                                   boost::bind(&AutoNavigation::sendPathCommand, this));

    udp.init(cfg->get_ip_addr(), cfg->get_port(), localPort, 0);
    strncpy((char *) sendBuf, "Auto", 5);
}

AutoNavigation::AutoNavigation(const std::string &cfg_path_)
{
    cfg = new ConfigParameters(cfg_path_);
    ifExternCfg = false;
    _running = false;
    loop_planning = LoopFunction("planning", 0.1, [this]
    { getPathCommand(); });
    loop_remoteSend = LoopFunction("sendRemote", cfg->get_send_period(),
                                   boost::bind(&AutoNavigation::sendPathCommand, this));

    udp.init(cfg->get_ip_addr(), cfg->get_port(), localPort, 0);

//    udp_sendImg.init("127.0.0.1", 6115, 0, 0);
//    udp_sendImg = RecvRequest();

//    udpTest.init("127.0.0.1", 6113, 0, 0);

    strncpy((char *) sendBuf, "Auto", 5);
}

AutoNavigation::~AutoNavigation()
{
    if (!ifExternCfg)
    {
        delete cfg;
    }
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
    auto path = path_planner.arc_planning(sourceX, sourceY, targetX, targetY, init_direcX, init_direcY, init_len, Rd, distance_threshold);

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
    if (cfg->show_map_image && udp_sendImg.sendingImg())
    {
        //    clock_gettime(CLOCK_MONOTONIC, &startTime);
        /************** Send new map info ***************/
        std::vector<uint8_t> newMapInfoBytes;
        uint8_t *tmp;
        uint32_t height = newMap.map.size();
        uint32_t width = newMap.map[0].size();
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
        auto path_bytes_len = ConvertArcPath2Bytes(path, pathBytes);
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
                tmpMap.push_back(static_cast<uint8_t>(newMap.map[i][j]));
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
        std::cout << "send_time: " << timePassed << " ms" << std::endl;
    }
/*******************************************************************************************************************/
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

    // start single lidar task
    th_lidar = std::thread(LidarMapThreadFun, ref(_running), ref(newMapQueue));
    std::cout << "[Task-Lidar] Start running!" << std::endl;

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
        udp.Send(sendBuf, AUTO_COMMAND_LENGTH);
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
