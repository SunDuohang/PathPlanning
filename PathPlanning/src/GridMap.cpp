/**
  ******************************************************************************
  * @file           : GridMap.cpp
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-1-25
  * @last_change    :
  ******************************************************************************
  */
//

#include "../include/GridMap.h"

GridMap::GridMap(const char *filename) {
    if (get_map_size(filename, self_width, self_height)){
        self_map = new int* [self_height];
        for(int i = 0; i < self_height; i++)
            self_map[i] = new int [self_width];
        load_map(filename, self_map, self_width, self_height);
    }
}

GridMap::~GridMap() {
    if (self_map){
        for(int i = 0; i < self_height; i++){
            delete []self_map[i];
        }
        delete []self_map;
        malloc_trim(0);
    }
}

GridMap::GridMap() {
    self_map = nullptr;
    self_height = -1;
    self_width = -1;
}

bool GridMap::set_GridMap(const char *filename) {
    if (self_map){
        for(int i = 0; i < self_height; i++){
            delete []self_map[i];
        }
        delete []self_map;
        malloc_trim(0);
    }

    if (get_map_size(filename, self_width, self_height)){
        self_map = new int* [self_height];
        for(int i = 0; i < self_height; i++)
            self_map[i] = new int [self_width];
        load_map(filename, self_map, self_width, self_height);
    }

}

void GridMap::operator= (const GridMap& map) {
    self_height = map.self_height;
    self_width = map.self_width;

    if (self_map){
        for(int i = 0; i < self_height; i++){
            delete []self_map[i];
        }
        delete []self_map;
        malloc_trim(0);
    }

    self_map = new int* [self_height];
    for(int i = 0; i < self_height; i++)
        self_map[i] = new int [self_width];

    for (int i = 0; i < self_height; i++){
        for(int j = 0; j < self_width; j++)
            self_map[i][j] = map.self_map[i][j];
    }

}

