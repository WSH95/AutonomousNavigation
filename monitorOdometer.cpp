//
// Created by wsh on 2021/12/21.
//

#include "AutoNavigation.h"
//#include <lcm/lcm-cpp.hpp>
//#include "lcm_monitor_odometer.hpp"

void AutoNavigation::monitorOdometerData()
{
    std::cout << "[1] lidar." << std::endl;
    std::cout << "[2] HWT101." << std::endl;
    std::cout << "[3] SE." << std::endl;
    std::cout << "[4] theta(HWT101) + xy(SE)." << std::endl;
    std::cout << "[5] lidar + HWT101." << std::endl;
    std::cout << "[6] lidar + SE." << std::endl;
    std::cout << "[7] lidar + SE + theta(HWT101)." << std::endl;
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
    _running = true;

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
                cfg->self_localization_mode = Self_localization_mode::replace_lidarTheta_HWT;
                readThetaFromHWT101.setup();
                fileNameHead = "theta_HWT_";
            }
            else if (num == 3)
            {
                cfg->self_localization_mode = Self_localization_mode::only_SE;
                fileNameHead = "se_";
            }
            else
            {
                cfg->self_localization_mode = Self_localization_mode::xy_SE_theta_HWT;
                readThetaFromHWT101.setup();
                fileNameHead = "xy_SE_theta_HWT_";
            }
        }
        else // need lidar thread as well
        {
            useLidar = true;
            if (num == 5)
            {
                cfg->self_localization_mode = Self_localization_mode::replace_lidarTheta_HWT;
                readThetaFromHWT101.setup();
                fileNameHead = "lidar+theta_HWT_";
            }
            else if (num == 6)
            {
                cfg->self_localization_mode = Self_localization_mode::only_SE;
                fileNameHead = "lidar+se_";
            }
            else
            {
                cfg->self_localization_mode = Self_localization_mode::xy_SE_theta_HWT;
                readThetaFromHWT101.setup();
                fileNameHead = "lidar+xy_SE+theta_HWT_";
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


//    lcm::LCM lcm("udpm://239.255.12.21:1221?ttl=255");
//    if (cfg->monitor_use_lcm)
//    {
//        if (!lcm.good())
//            return 1;
//    }
//    lcm_monitor_odometer monitorOdometer{};

    while (true)
    {
        std::vector<float> tmp_location;
        tmp_location.resize(3);

        if (useLidar)
            odometry_out_channel.pop_uptodate(newLidarOdometer);
        selfLocalizationQueue.pop_anyway(&tmp_location);

        monitorOdometer.odomLidar[0] = newLidarOdometer.transformDataSum[3];
        monitorOdometer.odomLidar[1] = newLidarOdometer.transformDataSum[4];
        monitorOdometer.odomLidar[2] = newLidarOdometer.transformDataSum[2];
        monitorOdometer.odomSE[0] = tmp_location[0];
        monitorOdometer.odomSE[1] = tmp_location[1];
        if ((num == 2) || (num == 4) || (num == 5) || (num == 7))
            monitorOdometer.thetaGyro = tmp_location[2];
        else
            monitorOdometer.odomSE[2] = tmp_location[2];

        if (cfg->monitor_use_lcm)
            lcmMonitor.publish("monitorOdometer", &monitorOdometer);

        if (cfg->monitor_save_txt)
        {
            outfile << monitorOdometer.odomLidar[0] << " ";
            outfile << monitorOdometer.odomLidar[1] << " ";
            outfile << monitorOdometer.odomLidar[2] << " ";
            outfile << monitorOdometer.odomSE[0] << " ";
            outfile << monitorOdometer.odomSE[1] << " ";
            outfile << monitorOdometer.odomSE[2] << " ";
            outfile << monitorOdometer.thetaGyro << std::endl;
        }

        if (!useLidar)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}

int main()
{
    AutoNavigation monitor;
    monitor.monitorOdometerData();

    return 0;
}