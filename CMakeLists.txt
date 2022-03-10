cmake_minimum_required(VERSION 3.5.0)
project(AutonomousNavigation)

#set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14 -pthread -O3 -Wall")
set(CMAKE_EXE_LINKER_FLAGS " -no-pie")
set(CMAKE_BUILD_TYPE "Release")

FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)

include_directories(
        Common/include
        ConfigParameters/include
        Navigation/include
        PathPlanning/include
        PathPlanning
#        yaml_cpp/include
#        Lidar/include
#        Lidar/RSDecoder
        Realtime-SingleLidar
        Realtime-SingleLidar/src
#        LinesDetection/include
)

link_directories(
#        yaml_cpp/lib
#        Lidar/lib
        Realtime-SingleLidar/lib
#        ${ZED_LIBRARY_DIR}
#        ${CUDA_LIBRARY_DIRS}
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories (${EIGEN3_INCLUDE_DIRS})

#[[option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif ()

if (COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
endif (COMMAND cmake_policy)

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif ()

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})]]

#link_directories(${ZED_LIBRARY_DIR})
#link_directories(${CUDA_LIBRARY_DIRS})


find_file(CONFIG_YAML_PATH path_planning_config.yaml ${PROJECT_SOURCE_DIR}/*)
#find_file(LIDAR_PARAM_PATH parameter.txt ${PROJECT_SOURCE_DIR}/Lidar/*)
#find_file(LIDAR_START_ICON_PATH start_15.jpg ${PROJECT_SOURCE_DIR}/Lidar/cyn_Param/*)
#find_file(CONFIG_END_ICON_PATH end_15.jpg ${PROJECT_SOURCE_DIR}/Lidar/cyn_Param/*)
configure_file(
        "${PROJECT_SOURCE_DIR}/ConfigParameters/include/config_yaml.h.in"
        "${PROJECT_SOURCE_DIR}/ConfigParameters/include/config_yaml.h"
)

# test map
#add_subdirectory(PathPlanning/testFile)

###################################### generate Lidar relative library ######################################
#add_subdirectory(Lidar)

file(GLOB_RECURSE SRC_FILES
        ${PROJECT_SOURCE_DIR}/Common/*.cpp
        ${PROJECT_SOURCE_DIR}/Navigation/*.cpp
        ${PROJECT_SOURCE_DIR}/PathPlanning/*.cpp
#        ${PROJECT_SOURCE_DIR}/Lidar/*.cpp
#        ${PROJECT_SOURCE_DIR}/LinesDetection/*.cpp
        )
message("SRC_FILES: ${SRC_FILES}")

# HWT101CT
include_directories(HWT101)

add_executable(auto_navigation main.cpp ${SRC_FILES})

#[[if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()]]

target_link_libraries(auto_navigation
#        LidarMap
#        PointCloudManage
#        grid_manage
        SingleLidarShare
#        yaml-cpp
        ${YAML_CPP_LIBRARIES}
        pcap
#        ${ZED_LIBS}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        -lm
        modbus)

include_directories(LcmPub/lcm_files/cpp)
add_executable(monitorOdometer monitorOdometer.cpp ${SRC_FILES})
target_link_libraries(monitorOdometer
        SingleLidarShare
        ${YAML_CPP_LIBRARIES}
        pcap
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        -lm
        modbus
        lcm)
