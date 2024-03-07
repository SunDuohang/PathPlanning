/**
  ******************************************************************************
  * @file           : test_gridmap.cpp
  * @author         : sunx
  * @brief          : 测试GridMap类，测试通过
  * @attention      : None
  * @date           : 24-3-4
  ******************************************************************************
  */

#include "../include/GridMap.h"
#include "../include/test_gridmap.h"

void test_gridmap(){
    const char* file_name = "../map_txt/Random_100x100_.3.txt";

    GridMap gridMap(file_name);
    int width = gridMap.self_width;
    int height = gridMap.self_height;
    std::cout << "width: " << width << " height: " << height << std::endl;
    if (gridMap.self_map.empty())
        std::cout << "Vector gridmap is empty" << std::endl;
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++)
            std::cout << gridMap.self_map[i][j] << "\t" ;
        std::cout << std::endl;
    }// Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)  // 这是因为没有成功的对self_map初始化
}