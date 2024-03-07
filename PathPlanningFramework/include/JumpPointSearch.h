/**
  ******************************************************************************
  * @file           : JumpPointSearch.h
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-6
  ******************************************************************************
  */
#pragma once
#ifndef PATHPLANNINGFRAMEWORK_JUMPPOINTSEARCH_H
#define PATHPLANNINGFRAMEWORK_JUMPPOINTSEARCH_H

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <malloc.h>
#include <fstream>
#include <sstream>
#include <fstream>
#include "./DataLoad.h"
#include "./DataStructure.h"
#include "./GridMap.h"

class JumpPointSearch{
private:
    GridMap gridMap;

    int offsets[8][2] = {{0, -1,},{1, 0}, {0, 1}, {-1, 0},
                          {1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

//    int offsets[32][2] = {{0, -1,},{1, 0}, {0, 1}, {-1, 0},
//                          {1, -1}, {1, 1}, {-1, 1}, {-1, -1},
//                          {1, -2}, {2, -1},{2, 1},{1, 2},
//                          {-1, 2}, {-2, 1}, {-2, -1}, {-1, -2},
//                          {1, -3}, {2, -3}, {3, -2}, {3, -1},
//                          {3, 1}, {3, 2}, {2, 3}, {1, 3},
//                          {-1, 3}, {-2, 3}, {-3, 2}, {-3, 1},
//                          {-3, -1}, {-3, -2}, {-2, -3}, {-1, -3}};

public:
    explicit JumpPointSearch(const char* name);
    JumpPointSearch()=default;
    ~JumpPointSearch()=default;
    std::pair<std::vector<Node >, double> run(const char* filename,const Node& start_point, const Node& target_point, const int method=8);
    std::vector<Node > get_neighbors(int** map, int width, int height, Node cur, int method);
    bool is_valid_neighbor(int** map, int width, int height, int x, int y);
    double heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor);
    double distance(int x1_cor, int y1_cor, int x2_cor, int y2_cor);
    std::pair<std::vector<Node >, double> construct_path(std::vector<Record_Item > close_list);
    static bool is_valid_point(int** map, int width, int height, int x, int y);
    Node* jump(int** map, int width, int height, int cur_x, int cur_y, int offset_x, int offset_y, int target_x, int target_y);
    int is_in_list(std::vector<Record_Item > list, Node cur_node);
};

#endif //PATHPLANNINGFRAMEWORK_JUMPPOINTSEARCH_H
