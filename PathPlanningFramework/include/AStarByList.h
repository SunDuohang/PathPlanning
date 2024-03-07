/**
  ******************************************************************************
  * @file           : AStarByList.h
  * @author         : sunx
  * @brief          : AStar By List
  * @attention      : None
  * @date           : 24-3-7
  ******************************************************************************
  */

#pragma once
#ifndef PATHPLANNINGFRAMEWORK_ASTARBYLIST_H
#define PATHPLANNINGFRAMEWORK_ASTARBYLIST_H

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
#include <ctime>
#include "./GridMap.h"
#include "./DataLoad.h"
#include "./DataStructure.h"

class AStarbyList{
    GridMap gridMap1;
    int map_width{};
    int map_height{};

    const int offsets[32][2] = {{0, -1,},{1, 0}, {0, 1}, {-1, 0},
                          {1, -1}, {1, 1}, {-1, 1}, {-1, -1},
                          {1, -2}, {2, -1},{2, 1},{1, 2},
                          {-1, 2}, {-2, 1}, {-2, -1}, {-1, -2},
                          {1, -3}, {2, -3}, {3, -2}, {3, -1},
                          {3, 1}, {3, 2}, {2, 3}, {1, 3},
                          {-1, 3}, {-2, 3}, {-3, 2}, {-3, 1},
                          {-3, -1}, {-3, -2}, {-2, -3}, {-1, -3}};

    const short new_offsets[32][9] = {{0,  -1, 0, 0,  0, 0,  0, 0, 1},
                                       {1,  0,  0, 0,  0, 0,  0, 0, 1},
                                       {0,  1,  0, 0,  0, 0,  0, 0, 1},
                                       {-1, 0,  0, 0,  0, 0,  0, 0, 1},
                                       {1,  -1, 0, 0,  0, 0,  0, 0, 1},
                                       {1,  1,  0, 0,  0, 0,  0, 0, 1},
                                       {-1, 1,  0, 0,  0, 0,  0, 0, 1},
                                       {-1, -1, 0, 0,  0, 0,  0, 0, 1},
                                       {0,  -1, 1, -1, 1, -2, 0, 0, 3},
                                       {1,  0,  1, -1, 2, -1, 0, 0, 3},
                                       {1,  0,  1, 1,  2, 1,  0, 0, 3},
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

    short change_offsets_obs[24][9] = {{-1, 0, -1, -1, -2, -1, -2, -2, 4},
                                       {-1, 0, -1, -1, -2, -1, 0, 0, 2},
                                       {-1, 0, -2, 1, 0, 0, 0, 0, 2},
                                       {0, -1, -1, -1, -1, -2, -2, -2, 4},
                                       {0, -1, -1, -1, 0, 0, 0, 0, 2},
                                       {0, -1, -1, -2, 0, 0, 0, 0, 2},
                                       {0, -1, 1, -1, 1, -2, 2, -2, 4},
                                       {0, -1, 1, -1, 0, 0, 0, 0, 2},
                                       {0, -1, 1, -2, 0, 0, 0, 0, 2},
                                       {1, 0, 1, -1, 2, -1, 2, -2, 4},
                                       {1, 0, 1, -1, 0, 0, 0, 0, 2},
                                       {1, 0, 2, -1, 0, 0, 0, 0, 2},
                                       {1, 0, 1, 1, 2, 1, 2, 2, 4},
                                       {1, 0, 1, 1, 0, 0, 0, 0, 2},
                                       {1, 0, 2, 1, 0, 0, 0, 0, 2},
                                       {0, 1, 1, 1, 1, 2, 2, 2, 4},
                                       {0, 1, 1, 1, 0, 0, 0, 0, 2},
                                       {0, 1, 1, 2, 0, 0, 0, 0, 2},
                                       {0, 1, -1, 1, -1, 2, -2, 2, 4},
                                       {0, 1, -1, 1, 0, 0, 0, 0, 2},
                                       {0, 1, -1, 2, 0, 0, 0, 0, 2},
                                       {-1, 0, -1, 1, -2, 1, -2, 2, 4},
                                       {-1, 0, -1, 1, 0, 0, 0, 0, 2},
                                       {-1, 0, -2, 1, 0, 0, 0, 0, 2}};


    short outside_offsets[24][2] = {{-3, -2}, {-2, -1}, {-3, -1},
                                    {-2, -3}, {-1, -2}, {-1, -3},
                                    {2, -3}, {1, -2}, {1, -3},
                                    {3, -2}, {2, -1}, {3, -1},
                                    {3, 2}, {2, 1}, {3, 1},
                                    {2, 3}, {1, 2}, {1, 3},
                                    {-2, 3}, {-1, 2}, {-1, 3},
                                    {-3, 2}, {-2, 1}, {-3, 1},};

    short change_offsets[17][2] = {{0, -1},{1, 0}, {0, 1}, {-1, 0},
                                   {1, -1}, {1, 1}, {-1, 1}, {-1, -1},
                                   {0, 0}, {0, 0}, {0, 0}, {0, 0},
                                   {0, 0}, {0, 0}, {0, 0}, {0, 0},
                                   {0, 0}};

    short change_labels[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

public:
    explicit AStarbyList(const char* filename);
    AStarbyList() = default;
    ~AStarbyList() = default;

    std::pair<std::vector<Node >, double> run(const char* filename, Node& start_point, Node& target_point, int method=16);

    std::pair<std::vector<Node >, double> run(const GridMap& gridMap, const Node& start_point, const Node& target_point, int method=16);

    std::vector<Node > get_neighbors(std::vector<std::vector<int> > &map, int width, int height, const Node& cur, int method=16);

    std::vector<Node > get_neighbors(std::vector<std::vector<int> > &map, int width, int height, const Node& cur, int dx, int dy, int method=16);

    std::vector<Node > get_neighbors_new(std::vector<std::vector<int> > map, int width, int height, const Node& cur, int method=16);

    std::vector<Node > get_neighbors_oritanted(std::vector<std::vector<int> > &map, int width, int height, Node& cur,const short flags[], int lens);

    static bool is_valid_neighbor(std::vector<std::vector<int> > &map, int width, int height, int x, int y);

//    bool is_valid_move_new(T1 **map, Node<T1> sp, Node<T1> ep);

    static bool is_valid_move(std::vector<std::vector<int> > &map,int width, int height, const Node& sp, const Node& ep);

    bool is_valid_move_new(std::vector<std::vector<int> > &map, Node& cur, int width, int height, int i);

    static double Euli_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    static double Manh_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    static double Cheb_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    static double Octi_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    static double distance(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    std::vector<Node > get_lines(Node& sp, Node& ep);

    static bool is_in_line(std::vector<Node >& line, Node p);

    static std::pair<std::vector<Node >, double> construct_path(std::vector<Record_Item >& close_list);

    static int is_in_list(std::vector<Record_Item >& list, const Node& cur_node);

    static int get_target_angle_seg(const Node & sp, const Node & ep);

    void set_changeable_offsets(const Node & sp, const Node & ep);
};

#endif //PATHPLANNINGFRAMEWORK_ASTARBYLIST_H
