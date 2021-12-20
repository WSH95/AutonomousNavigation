cmake_minimum_required(VERSION 3.5.0)

include_directories(test_map.h.in test_map.h)
find_file(TEST_MAP_PATH combined_grid.txt .)
configure_file(
        "${PROJECT_SOURCE_DIR}/PathPlanning/testFile/test_map.h.in"
        "${PROJECT_SOURCE_DIR}/PathPlanning/testFile/test_map.h"
)
