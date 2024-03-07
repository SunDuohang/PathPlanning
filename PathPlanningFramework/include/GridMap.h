/**
  ******************************************************************************
  * @file           : GridMap.h
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-4
  * @last_change    :
  ******************************************************************************
  */

#ifndef PATHPLANNINGFRAMEWORK_GRIDMAP_H
#define PATHPLANNINGFRAMEWORK_GRIDMAP_H

#include <utility>
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
    GridMap(std::vector<std::vector<int> > grid, int MapWidth, int MapHeight): self_map(std::move(grid)), self_width(MapWidth), self_height(MapHeight){};
    GridMap();


    bool set_GridMap(const char* filename);

    void operator=(const GridMap& map);

    //int*** self_map;
    std::vector<std::vector<int> > self_map;
    int self_width{};
    int self_height{};
};

#endif //PATHPLANNINGFRAMEWORK_GRIDMAP_H
