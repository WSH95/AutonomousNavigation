//
// Created by wsh on 2021/8/25.
//

#include "AStar.hpp"
//#include "showImg.hpp"

//// test map
//#include "testFile/grid.h"
//#include "testFile/test_map.h"

#include <iostream>
#include <sys/time.h>
#include <functional>

#include "AutoNavigation.h"
#include "config_yaml.h"
//#include "MutiLidar.h"

//#include "RecvPlanningUdp.hpp"
#include "UDP.hpp"


using namespace std;

//string paramAdress = "../Lidar/cyn_Param/parameter.txt";
//string startIconAdrress = "../Lidar/cyn_Param/icon/start_15.jpg";
//string endIconAdrress = "../Lidar/cyn_Param/icon/end_15.jpg";

int main()
{
//    timespec startTime;
//    timespec endTime;
//    clock_gettime(CLOCK_MONOTONIC, &startTime);
//    startTime.tv_sec = 0;
//    clock_gettime(CLOCK_MONOTONIC, &endTime);
//    auto lastPeriodTime = (float) ((endTime.tv_nsec - startTime.tv_nsec) / 1.e9 +
//                                   (endTime.tv_sec - startTime.tv_sec));
//
//    cout << lastPeriodTime << endl;
//
//    struct timeval start, end;
//    double diff;

    AutoNavigation na(CONFIG_YAML_PATH);
//    na.getPathCommand();
    na.start();

//    gettimeofday(&start, NULL);
//
//    for (int i = 0; i < 1; i++)
//    {
//        na.udpSend(true, 0, 0, 0);
//    }
//    gettimeofday(&end, NULL);
//    diff = 1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
//    std::cout << "Time is " << diff << "us" << endl;


    //////AStar::Vec2d source = {14, 69};
    //AStar::Vec2d target = {51, 104};

    //AStar::Vec2d target = {146, 105};

    //AStar::Vec2d source = {120, 120};
    //////AStar::Vec2d target = {100, 50};
    //AStar::Vec2d target = {140, 110};
    //AStar::Vec2d target = {15, 113};

//AStar::Vec2d source = {111, 46};
//AStar::Vec2d target = {121, 69};




//
//    GridMap GridFileReadTest(TEST_MAP_PATH);
//
//
//    gettimeofday(&start, NULL);
//    //AStar::Planning planning;
//
//    //planning.setGridMap(GridFileReadTest.GetGridVector());
//
//
//    //auto path = planning.findPath(source, target);
//
//
//    //auto path2 = planning.simplePath(path);
//
//    auto grid = GridFileReadTest.GetGridVectorTest();
//    std::cout << "row: " << grid.size() << std::endl << "col: " << grid[0].size() << std::endl;
//
//
//    auto smoothPath1 = AStar::path_planning(grid, 14, 69, 51, 104, 0, 1, 10, 20.0);
////    auto raw_path = AStar::raw_path(grid, 14, 69, 51, 104, 0, 1, 10, false);
//    auto cmd = AStar::getPathCommand(grid, 14, 69, 51, 104);
//
////    AStar::ArcPlanning a(grid);
////    auto path2 = a.arc_planning(14, 69, 51, 104, 1, 0, 5, 20.0);
//    gettimeofday(&end, NULL);
//    AStar::print_path(smoothPath1);
////    AStar::print_path(path2);
//
//
//
////    AStar::Arc aa;
////    AStar::VecVecD bb = aa.ArcSmooth(path2, 24);
//
//    //planning.showPath(path2);
//
//    //showImage(GridFileReadTest.GetGridVector(), path2, 8);
///*
//    Image img(GridFileReadTest.GetGridVector(), 12, path2, bb);
//    img.img_line();
//    img.img_bspline();
//    img.img_arc();
//    img.img_show();
//
//*/
//    //std::cout << "pointNum: " << smoothPath.size() << endl;
//    diff = 1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
//    std::cout << "Time is " << diff << "us" << endl;

    while(true)
    {
        sleep(1);
    }

    return 0;
}

