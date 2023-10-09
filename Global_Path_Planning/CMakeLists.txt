cmake_minimum_required(VERSION 3.10)
project(Global_Path_Planning)

set(CMAKE_CXX_STANDARD 17)

# 引入GDAL
find_package(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIRS})

message(STATUS "GDAL library status:")
message(STATUS "    version: ${GDAL_VERSION}")
message(STATUS "    libraries: ${GDAL_LIBRARY}")
message(STATUS "    include path: ${GDAL_INCLUDE_DIRS}")

add_executable(Global_Path_Planning main.cpp src/CoorTransform.cpp include/CoorTransform.h include/MapLoad.h src/MapLoad.cpp src/MapLoad.h app/main.h)
