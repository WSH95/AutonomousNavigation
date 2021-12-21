//
// Created by wsh on 2021/12/21.
//

#include "AutoNavigation.h"
#include <lcm/lcm-cpp.hpp>
#include "lcm_monitor_odometer.hpp"

int AutoNavigation::monitorOdometerData()
{
    std::cout << "[1] lidar." << std::endl;
    std::cout << "[2] theta (from HWT101)." << std::endl;
    std::cout << "[3] xy (from NUC)." << std::endl;
    std::cout << "[4] theta + xy." << std::endl;
    std::cout << "[5] lidar + theta." << std::endl;
    std::cout << "[6] lidar + xy." << std::endl;
    std::cout << "[7] lidar + theta + xy." << std::endl;
    std::cout << std::endl;
    std::cout << "Please choose a integer number from 1 to 7 indicating what data you want to monitor: ";
    int num;
    std::cin >> num;
    while ((num < 1) || (num > 7))
    {
        std::cout << "[Input Error] Please enter the number from 1 to 7." << std::endl;
        std::cout << "Please choose a integer number from 1 to 7 indicating what data you want to monitor: ";
        std::cin >> num;
    }
    std::cout << std::endl;

    // 获取当前时间，精确到秒
    time_t currentTime = std::time(NULL);
    char chCurrentTime[64];
    std::strftime(chCurrentTime, sizeof(chCurrentTime), "%Y-%m-%d_%H-%M-%S", std::localtime(&currentTime)); //年月日 时分秒
    std::string stCurrentTime = chCurrentTime;// 转为string

    std::string fileNameHead, fileName;

    bool useLidar = false;

    if (num == 1) // only need lidar thread
    {
        getParameter(setParam);
        th_lidar = std::thread(AutonomousNav, ref(_running), ref(setParam), ref(newMapQueue), ref(odometry_out_channel),
                               ref(featureExtra_out_channel));
        useLidar = true;
        fileNameHead = "lidar_";
    }
    else
    {
        if ((num == 2) || (num == 3) || (num == 4)) // no need lidar thread
        {
            if (num == 2)
            {
                cfg->self_localization_mode = Self_localization_mode::replace_lidarTheta;
                readThetaFromHWT101.setup();
                fileNameHead = "theta_";
            }
            else if (num == 3)
            {
                cfg->self_localization_mode = Self_localization_mode::replace_lidarXY;
                fileNameHead = "xy_";
            }
            else
            {
                cfg->self_localization_mode = Self_localization_mode::no_lidar;
                readThetaFromHWT101.setup();
                fileNameHead = "theta+xy_";
            }
        }
        else // need lidar thread as well
        {
            useLidar = true;
            if (num == 5)
            {
                cfg->self_localization_mode = Self_localization_mode::replace_lidarTheta;
                readThetaFromHWT101.setup();
                fileNameHead = "lidar+theta_";
            }
            else if (num == 6)
            {
                cfg->self_localization_mode = Self_localization_mode::replace_lidarXY;
                fileNameHead = "lidar+xy_";
            }
            else
            {
                cfg->self_localization_mode = Self_localization_mode::no_lidar;
                readThetaFromHWT101.setup();
                fileNameHead = "lidar+theta+xy_";
            }

            getParameter(setParam);
            th_lidar = std::thread(AutonomousNav, ref(_running), ref(setParam), ref(newMapQueue),
                                   ref(odometry_out_channel),
                                   ref(featureExtra_out_channel));
        }

        loop_selfLocalizationWOLidar.start();
    }

    fileName = fileNameHead + stCurrentTime + ".txt";

    ofstream outfile;
    if (cfg->monitor_save_txt)
        outfile.open("../data/" + fileName, ios::out | ios::trunc);


    lcm::LCM lcm("udpm://239.255.12.21:1221?ttl=255");
    if (cfg->monitor_use_lcm)
    {
        if (!lcm.good())
            return 1;
    }
    lcm_monitor_odometer monitorOdometer{};

    while (true)
    {
        std::vector<float> tmp_location;
        tmp_location.resize(3);

        if (useLidar)
            odometry_out_channel.pop_uptodate(newLidarOdometer);
        selfLocalizationQueue.pop_anyway(&tmp_location);

        monitorOdometer.xLidar = newLidarOdometer.transformDataSum[3];
        monitorOdometer.yLidar = newLidarOdometer.transformDataSum[4];
        monitorOdometer.thetaLidar = newLidarOdometer.transformDataSum[2];
        monitorOdometer.xSE = tmp_location[0];
        monitorOdometer.ySE = tmp_location[1];
        monitorOdometer.thetaGyro = tmp_location[2];

        if (cfg->monitor_use_lcm)
            lcm.publish("monitorOdometer", &monitorOdometer);

        if (cfg->monitor_save_txt)
        {
            outfile << monitorOdometer.xLidar << " ";
            outfile << monitorOdometer.yLidar << " ";
            outfile << monitorOdometer.thetaLidar << " ";
            outfile << monitorOdometer.xSE << " ";
            outfile << monitorOdometer.ySE << " ";
            outfile << monitorOdometer.thetaGyro << std::endl;
        }
    }

    return 0;
}

int main()
{
    AutoNavigation monitor;
    monitor.monitorOdometerData();

    return 0;
}