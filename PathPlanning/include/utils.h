//
// Created by sunx on 24-1-11.
//

#ifndef PATHPLANNING_UTILS_H
#define PATHPLANNING_UTILS_H

#include <opencv2/opencv.hpp>
#include "DataStructure.hpp"
#include <iostream>
#include <vector>
#include <string>


namespace utils
{
    /**
     * @brief 地形+路径可视化
     *
     * @param path 路径点[x0,y0,x1,y1...]
     * @param len 路径长度
     * @param map 速度地图数据[数据只可以有0, 1， 2， ...整数型]
     * @param map_size_wh 地图宽高
     * @param img_save_path 地图保存路径 例如: ../output/dstImg.png
     * @return cv::Mat 生成的地图
     */
    cv::Mat visualizePath(int* path, int len, double* map, double* map_size_wh, std::string img_save_path="../output/dstImg.png");

    /**
     * @brief go/nogo地图可视化
     *
     * @param path
     * @param len
     * @param map
     * @param map_size_wh
     * @param img_save_path
     * @return cv::Mat
     */
    //cv::Mat visualizePath(int* path, int len, bool* map, int* map_size_wh, std::string img_save_path="../output/dstImg.png");
    cv::Mat visualizePath(int* path, int len, int** map, int map_xsize, int map_ysize, const char* filename);

//    template <class T>
//    cv::Mat visualizePath(std::vector<Node<T> > path, int** map, int map_xsize, int map_ysize, std::string img_save_path="../output/dstImg.png");
}

#endif //PATHPLANNING_UTILS_H
