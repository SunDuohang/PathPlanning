/**
  ******************************************************************************
  * @file           : test_dataload.cpp
  * @author         : sunx
  * @brief          : 测试 Dataload中的函数
  * @attention      : None
  * @date           : 24-3-5
  ******************************************************************************
  */

#include "../include/DataLoad.h"
#include <iostream>

void test_DataLoad(){
    const char* filename = "../map_txt/test_map.txt";
    int map_width;
    int map_height;
    int **map;
    int result = 0;
    result = get_map_size(filename, map_width, map_height);
    if(result==-1){
        std::cout << "File open failure" << std::endl;
        return ;
    }
    map = new int* [map_height];
    for(int i = 0; i < map_height; i++)
        map[i] = new int [map_width];
    std::cout << filename << " width: " << map_width << " height: " << map_height << std::endl;

    load_map(filename, map, map_width, map_height);
    for(int i = 0; i < map_height; i++){
        for(int j = 0; j < map_width; j++)
            std::cout << map[i][j] << " ";
        std::cout << std::endl;
    }

    for(int i = 0; i < map_height; i++){
        delete []map[i];
    }
    delete []map;
}