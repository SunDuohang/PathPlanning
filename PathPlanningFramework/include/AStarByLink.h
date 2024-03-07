/**
  ******************************************************************************
  * @file           : AStarByLink.h
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-7
  ******************************************************************************
  */

#pragma once
#ifndef PATHPLANNINGFRAMEWORK_ASTARBYLINK_H
#define PATHPLANNINGFRAMEWORK_ASTARBYLINK_H

#include <utility>
#include <vector>
#include <list>
#include <memory>
#include <algorithm>
#include <iostream>
#include "./DataStructure.h"
#include "./GridMap.h"



class AStarByLink{
private:
    GridMap gridMap;
    int map_width;
    int map_height;

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
    explicit AStarByLink(const char* filename);
    AStarByLink() = default;
    ~AStarByLink() = default;

    std::pair<std::vector<Node >, double> run(const char* filename, Node& start_point, Node& target_point, int method=16);

    std::vector<Node > get_neighbors(std::vector<std::vector<int> > &map, int width, int height, std::shared_ptr<ANode> cur, int method=16);

    std::vector<Node > get_neighbors(std::vector<std::vector<int> > &map, int width, int height, std::shared_ptr<ANode> cur, int dx, int dy, int method=16);

    std::pair<std::vector<Node >, double> construct_path(std::shared_ptr<ANode> node);

    static bool is_valid_neighbor(std::vector<std::vector<int> > &map, int width, int height, int x, int y);

    static bool is_valid_move(std::vector<std::vector<int> > &map, int width, int height, const Node& sp, const Node& ep);

    bool is_valid_move_new(std::vector<std::vector<int> > &map, Node cur, int width, int height, int i);

private:

    static double Euli_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    static double Manh_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    static double Cheb_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    static double Octi_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor);

    static double distance(int x1_cor, int y1_cor, int x2_cor, int y2_cor);


};



#endif //PATHPLANNINGFRAMEWORK_ASTARBYLINK_H
