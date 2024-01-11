/**
  ******************************************************************************
  * @file           : loadmap.h
  * @author         : SUNX
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/19
  * @last_change    : 
  ******************************************************************************
  */


#pragma once
#ifndef PATHPLANNING_LOADMAP_H
#define PATHPLANNING_LOADMAP_H

struct TestEOL {
    bool operator()(char c) {
        last = c;
        return last == '\n';
    }
    char last;
};
/**
 * @brief
 * @param filename const char* filename
 * @param xSize int
 * @param ySize int
 * @return
 */
int get_map_size(const char* filename, int &xSize, int &ySize);
/**
 * @brief read txt as matrix
 * @param filename const char* filename
 * @param map 2d int matrix
 * @param xSize int
 * @param ySize int
 */
void load_map(const char* filename, int** map, int xSize, int ySize);

//void load_map(const char* filename,int* map[], int xSize, int ySize);

#endif //PATHPLANNING_LOADMAP_H
