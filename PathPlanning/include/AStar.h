/**
  ******************************************************************************
  * @file           : AStar.h
  * @author         : SUNX
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/19
  * @last_change    : 
  ******************************************************************************
  */


#pragma once
#ifndef PATHPLANNING_ASTAR_H
#define PATHPLANNING_ASTAR_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <cmath>
#include <cstring>
#include <memory>
#include <malloc.h>
#include <fstream>
#include <sstream>
#include "time.h"
#include "./GridMap.h"
#include "./DataLoad.h"
#include "./DataStructure.h"

class AStar{
private:
    int** self_map;
    int self_width{};
    int self_height{};

    int offsets[32][2] = {{0, -1,},{1, 0}, {0, 1}, {-1, 0},
                          {1, -1}, {1, 1}, {-1, 1}, {-1, -1},
                          {1, -2}, {2, -1},{2, 1},{1, 2},
                          {-1, 2}, {-2, 1}, {-2, -1}, {-1, -2},
                          {1, -3}, {2, -3}, {3, -2}, {3, -1},
                          {3, 1}, {3, 2}, {2, 3}, {1, 3},
                          {-1, 3}, {-2, 3}, {-3, 2}, {-3, 1},
                          {-3, -1}, {-3, -2}, {-2, -3}, {-1, -3}};

    short new_offsets[32][9] = {{0, -1, 0, 0, 0, 0, 0, 0, 1},
                                {1, 0, 0, 0, 0, 0, 0, 0, 1},
                                {0, 1, 0, 0, 0, 0, 0, 0, 1},
                                {-1, 0, 0, 0, 0, 0, 0, 0, 1},
                                {1, -1, 0, 0, 0, 0, 0, 0, 1},
                                {1, 1, 0, 0, 0, 0, 0, 0, 1},
                                {-1, 1, 0, 0, 0, 0, 0, 0, 1},
                                {-1, -1, 0, 0, 0, 0, 0, 0, 1},
                                {0, -1, 1, -1, 1, -2, 0, 0, 3},
                                {1, 0, 1, -1, 2, -1, 0, 0, 3},
                                {1, 0, 1, 1, 2, 1, 0, 0, 3},
                                {0, 1, 1, 1, 1, 2, 0, 0, 3},
                                {0, 1, -1, 1, -1, 2, 0, 0, 3},
                                {-1, 0, -1, 1, -2, 1, 0, 0, 3},
                                {-1, 0, -1, -1, -2, -1, 0, 0, 3},
                                {0, -1, -1, -1, -1, -2, 0, 0, 3},
                                {0, -1, 1, -2, 1, -3, 0, 0, 3},
                                {0, -1, 1, -1, 1, -2, 2, -2, 4},
                                {1, 0, 1, -1, 2, -1, 2, -2, 4},
                                {1, 0, 2, -1, 3, -1, 0, 0, 3},
                                {1, 0, 2, 1, 3, 1, 0, 0, 3},
                                {1, 0, 1, 1, 2, 1, 2, 2, 4},
                                {0, 1, 1, 1, 1, 2, 2, 2, 4},
                                {0, 1, 1, 2, 1, 3, 0, 0, 3},
                                {0, 1, -1, 2, -1, 3, 0, 0, 3},
                                {0, 1, -1, 1, -1, 2, -2, 2, 4},
                                {-1, 0, -1, 1, -2, 1, -2, 2, 4},
                                {-1, 0, -2, 1, -3, 1, 0, 0, 3},
                                {-1, 0, -2, -1, -3, -1, 0, 0, 3},
                                {-1, 0, -1, -1, -2, -1, -2, -2, 4},
                                {0, -1, -1, -1, -1, -2, -2, -2, 4},
                                {0, -1, -1, -2, -1, -3, 0, 0, 3}};


public:
    explicit AStar(const char* filename);
    AStar() = default;
    ~AStar();

    std::pair<std::vector<Node >, double> run(const char* filename, Node start_point, Node target_point, int method=16);

    std::pair<std::vector<Node >, double> run(const GridMap& gridMap, const Node& start_point, const Node& target_point, int method=16);

    std::vector<Node > get_neighbors(int** map, int map_xsize, int map_ysize, Node cur,int method=16);

    std::vector<Node > get_neighbors_new(int** map, int map_xsize, int map_ysize, const Node& cur,int method=16);

    bool is_valid_neighbor(int x, int y, int map_xsize, int map_ysize, int** map);

//    bool is_valid_move_new(T1 **map, Node<T1> sp, Node<T1> ep);

    bool is_valid_move(int **map, Node sp, Node ep, int map_xsize, int map_ysize);

    bool is_valid_move_new(int **map, Node cur, int map_xsize, int map_ysize, int i);

    double heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    double distance(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    std::vector<Node > get_lines(Node sp, Node ep);

    bool is_in_line(std::vector<Node > line, Node p);

    std::pair<std::vector<Node >, double> construct_path(std::vector<Record_Item > close_list);

    int is_in_list(std::vector<Record_Item > close_list, Node cur_node);

    bool is_in_open_list(std::vector<Open_list_Item > open_list, Node cur_node);
};




#endif //PATHPLANNING_ASTAR_H
