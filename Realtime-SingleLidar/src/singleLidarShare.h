//
// Created by wsh on 2021/11/18.
//

#ifndef AUTONOMOUS_NAVIGATION_SINGLELIDARSHARE_H
#define AUTONOMOUS_NAVIGATION_SINGLELIDARSHARE_H

#include "utils/lidarMap_t.h"

void LidarMapThreadFun(volatile bool &RunFlag, LidarMap_t& lidarMap_th);

#endif //AUTONOMOUS_NAVIGATION_SINGLELIDARSHARE_H
