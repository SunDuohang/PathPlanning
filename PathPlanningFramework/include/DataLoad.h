/**
  ******************************************************************************
  * @file           : DataLoad.h
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-4
  ******************************************************************************
  */

#pragma once
#ifndef PATHPLANNINGFRAMEWORK_DATALOAD_H
#define PATHPLANNINGFRAMEWORK_DATALOAD_H
#include <iostream>
#include <vector>

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

bool get_map_size(const char *filename, int &width, int &height);

/**
 * @brief read txt as matrix std::vector<std::vector<int>>
 * @param filename const char* filename
 * @param map 2d int matrix
 * @param xSize int
 * @param ySize int
 */
void load_map(const char *filename, int **map, int width, int height);

/**
 * @brief
 * @param filename
 * @param map
 * @param width
 * @param height
 */
void load_data(const char *filename, std::vector<std::vector<int> > &data);

/**
 * @brief 读入数据到vector
 * @param filename
 * @param data
 */

#endif //PATHPLANNINGFRAMEWORK_DATALOAD_H
