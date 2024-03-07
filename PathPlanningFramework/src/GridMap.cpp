/**
  ******************************************************************************
  * @file           : GridMap.cpp
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-4
  ******************************************************************************
  */

#include "../include/GridMap.h"

GridMap::GridMap(const char *filename) {
    if (get_map_size(filename, self_width, self_height)){
        load_data(filename, self_map);
    }
}

GridMap::~GridMap() {
    if (!self_map.empty()){
        for(int i = 0; i < self_height; i++){
            self_map[i].clear();
            self_map[i].shrink_to_fit();
        }
        self_map.clear();
        self_map.shrink_to_fit();
        malloc_trim(0);
    }
}

GridMap::GridMap() {
    self_height = -1;
    self_width = -1;
}

bool GridMap::set_GridMap(const char *filename) {
    if (!self_map.empty()){
        for(int i = 0; i < self_height; i++){
            self_map[i].clear();
            self_map[i].shrink_to_fit();
        }
        self_map.clear();
        self_map.shrink_to_fit();
        malloc_trim(0);
    }

    if (get_map_size(filename, self_width, self_height)){
        load_data(filename, self_map);
        return true;
    }
    return false;
}

void GridMap::operator= (const GridMap& map) {
    self_height = map.self_height;
    self_width = map.self_width;

    if (!self_map.empty()){
        for(int i = 0; i < self_height; i++){
            self_map[i].clear();
            self_map.shrink_to_fit();
        }
        self_map.clear();
        self_map.shrink_to_fit();
        malloc_trim(0);
    }
    self_map = map.self_map;
}
