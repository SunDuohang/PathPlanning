/**
  ******************************************************************************
  * @file           : JumpPointSearch.cpp
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-6
  ******************************************************************************
  */

#include "../include/JumpPointSearch.h"

JumpPointSearch::JumpPointSearch(const char *filename) {
    gridMap.set_GridMap(filename);
}

std::pair<std::vector<Node>, double>
JumpPointSearch::run(const char *filename, const Node& start_point, const Node& target_point, const int method){
    GridMap map(filename);
    int width = map.self_width;
    int height = map.self_height;

    std::vector<Record_Item > close_list;   // close_list
    std::vector<Record_Item > open_list;    // open_list

    double g_value = 0.0;


}