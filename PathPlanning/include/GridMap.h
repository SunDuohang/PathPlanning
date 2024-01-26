/**
  ******************************************************************************
  * @file           : GridMap.h
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-1-25
  * @last_change    :
  ******************************************************************************
  */
//

#ifndef PATHPLANNING_GRIDMAP_H
#define PATHPLANNING_GRIDMAP_H
#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <cmath>
#include <cstring>
#include <memory>
#include <malloc.h>
#include "./DataLoad.h"
#include "./DataStructure.h"

class GridMap{
public:
    explicit GridMap(const char* filename);
    ~GridMap();
    GridMap(int** grid, int MapWidth, int MapHeight): self_map(grid), self_width(MapWidth), self_height(MapHeight){};
    GridMap();

    bool set_GridMap(const char* filename);

    void operator=(const GridMap& map);

    int** self_map;
    int self_width{};
    int self_height{};
};

#endif //PATHPLANNING_GRIDMAP_H
