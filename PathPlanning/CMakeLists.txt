cmake_minimum_required(VERSION 3.22)
project(PathPlanning)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

#set(OpenCV_DIR /usr/share/OpenCV/)  # 添加OPENCV库
find_package(OpenCV REQUIRED)

#include_directories(D:/Tool/boost_1_81_0)
#link_directories(D:/Tool/boost_1_81_0/lib64-msvc-14.3)


add_library(module_global_path_planning
        include/AStar.hpp
        include/utils.h
        src/utils.cpp
        include/loadmap.h src/loadmap.cpp include/Dict.hpp include/List.hpp include/Elem.hpp
        include/Conv.h
        src/Conv.cpp
        include/DataStructure.hpp)

target_link_libraries(module_global_path_planning PUBLIC ${OpenCV_LIBS} )

# target_link_libraries(PathPlanning PUBLIC ${OpenCV_LIBS} module_global_path_planning)

add_executable(PathPlanning ./src/main.cpp)
target_link_libraries(PathPlanning PUBLIC ${OpenCV_LIBS} module_global_path_planning)

