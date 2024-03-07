cmake_minimum_required(VERSION 3.22)
project(PathPlanningFramework)

set(CMAKE_BUILD_TYPE Release)   # CMAKE类型为
set(CMAKE_CXX_STANDARD 17)

set(OpenCV_Dir /usr/share/OpenCV)   # 添加OpenCV库

add_executable(PathPlanningFramework
        src/main.cpp
        include/DataLoad.h
        src/DataLoad.cpp
        include/DataStructure.h
        include/GridMap.h
        src/GridMap.cpp
        include/test_dataload.h
        src/test_dataload.cpp
        include/test_gridmap.h
        src/test_gridmap.cpp
        include/AStar.h
        src/AStar.cpp
        include/AStarByList.h
        src/AStarByList.cpp
        include/AStarByLink.h
        src/AStarByLink.cpp
)

TARGET_LINK_LIBRARIES(PathPlanningFramework ${OpenCV_LIBS})
