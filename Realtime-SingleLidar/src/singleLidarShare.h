//
// Created by wsh on 2021/11/18.
//

#ifndef AUTONOMOUS_NAVIGATION_SINGLELIDARSHARE_H
#define AUTONOMOUS_NAVIGATION_SINGLELIDARSHARE_H

#include "utils/lidarMap_t.h"
#include "utils/channel.h"
#include "paramFile/paramParse.h"

void LidarMapThreadFun(volatile bool &RunFlag, const InitParams &GlobalParams, LidarMap_t &lidarMap_th);

void AutonomousNav(volatile bool &RunFlag, const InitParams &GlobalParams, LidarMap_t &lidarMap_th,
                   Channel<OdometryOut> &odometry_out_channel, Channel<ExtractionOut> &featureExtra_out_channel);

#endif //AUTONOMOUS_NAVIGATION_SINGLELIDARSHARE_H
