/**
  ******************************************************************************
  * @file           : utils.cpp
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-1-12
  * @last_change    :
  ******************************************************************************
  */

#include "../include/utils.h"

cv::Mat utils::visualizePath(int* path, int len, double* map, double* map_size_wh, std::string img_save_path)
{
    // 颜色配置
    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Vec3i(128, 128, 128)); // 不可通行区域 灰色
    colors.push_back(cv::Vec3i(0, 69, 255));    //田地：橘红
    colors.push_back(cv::Vec3i(0, 165, 255));   //草地
    colors.push_back(cv::Vec3i(0, 255, 0));     // 空地：浅绿
    colors.push_back(cv::Vec3i(0, 150, 0));     // 道路：深绿
    colors.push_back(cv::Vec3i(0, 255, 0));
    colors.push_back(cv::Vec3i(0, 255, 194));
    colors.push_back(cv::Vec3i(0, 255, 220));
    colors.push_back(cv::Vec3i(0, 255, 254));
    colors.push_back(cv::Vec3i(247, 11, 254));
    colors.push_back(cv::Vec3i(0, 0, 255));
    colors.push_back(cv::Vec3i(0, 0, 255));
    colors.push_back(cv::Vec3i(128, 128, 128));

    cv::Mat rawImg = cv::Mat::zeros(map_size_wh[1], map_size_wh[0], CV_8UC3);
    for (int i = 0; i < int(map_size_wh[1]); i++)
        for (int j = 0; j < int(map_size_wh[0]); j++)
        {
            rawImg.at<cv::Vec3b>(i, j)[0] = colors[map[i * int(map_size_wh[0]) + j]][0];
            rawImg.at<cv::Vec3b>(i, j)[1] = colors[map[i * int(map_size_wh[0]) + j]][1];
            rawImg.at<cv::Vec3b>(i, j)[2] = colors[map[i * int(map_size_wh[0]) + j]][2];
        }

    for (int i = 0; i < len; i++)
        cv::circle(rawImg, cv::Point2i(path[i*2], path[i*2+1]),0, cv::Scalar(255, 0, 0), -1);
    cv::imwrite(img_save_path, rawImg);
    return rawImg;
}


cv::Mat utils::visualizePath(int* path, int len, bool* map, int* map_size_wh, std::string img_save_path)
{
    // 颜色配置
    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Vec3i(128, 128, 128)); // 不可通行区域 灰色
    colors.push_back(cv::Vec3i(255, 255, 255));    //可通行区域白色

    cv::Mat rawImg = cv::Mat::zeros(map_size_wh[1], map_size_wh[0], CV_8UC3);
    for (int i = 0; i < int(map_size_wh[1]); i++)
        for (int j = 0; j < int(map_size_wh[0]); j++)
        {
            rawImg.at<cv::Vec3b>(i, j)[0] = colors[map[i * int(map_size_wh[0]) + j]][0];
            rawImg.at<cv::Vec3b>(i, j)[1] = colors[map[i * int(map_size_wh[0]) + j]][1];
            rawImg.at<cv::Vec3b>(i, j)[2] = colors[map[i * int(map_size_wh[0]) + j]][2];
        }
    cv::imwrite("../output/raw.png", rawImg);
    for (int i = 0; i < len; i++)
    {
        cv::circle(rawImg, cv::Point2i(path[i*2], path[i*2+1]),3, cv::Scalar(255, 0, 0), -1);
    }

    cv::imwrite(img_save_path, rawImg);
    return rawImg;
}

cv::Mat utils::visualizeMap(int** map, int map_width, int map_height, std::string img_save_path)
{
    // 颜色配置
    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Vec3i(255, 255, 255));    //可通行区域白色
    colors.push_back(cv::Vec3i(0, 0, 0)); // 不可通行区域 灰色

    // cv::Mat img(height, width, date_type, cv::Scalar())
    cv::Mat rawImg = cv::Mat::zeros(map_height, map_width, CV_8UC3);
    for (int i = 0; i < map_height; i++)
        for (int j = 0; j < map_width; j++)
        {
            rawImg.at<cv::Vec3b>(i, j)[0] = colors[map[i][j]][0];
            rawImg.at<cv::Vec3b>(i, j)[1] = colors[map[i][j]][1];
            rawImg.at<cv::Vec3b>(i, j)[2] = colors[map[i][j]][2];
        }
//    cv::Mat rawImg(map_height, map_height, CV_8UC3, cv::Scalar(255, 255, 255));
//    cv::rectangle(rawImg, cv::Point2i ())
    cv::imwrite(img_save_path, rawImg);
    return rawImg;
};

cv::Mat utils::visualizePath(int* path, int len, int** map, int map_width, int map_height, std::string img_save_path){
    // 颜色配置
    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Vec3i(255, 255, 255));    //可通行区域白色
    colors.push_back(cv::Vec3i(0, 0, 0)); // 不可通行区域 灰色

    // cv::Mat img(height, width, date_type, cv::Scalar())
    cv::Mat rawImg = cv::Mat::zeros(map_height, map_width, CV_8UC3);
    for (int i = 0; i < map_height; i++)
        for (int j = 0; j < map_width; j++){
            rawImg.at<cv::Vec3b>(i, j)[0] = colors[map[i][j]][0];
            rawImg.at<cv::Vec3b>(i, j)[1] = colors[map[i][j]][1];
            rawImg.at<cv::Vec3b>(i, j)[2] = colors[map[i][j]][2];
        }

    for (int i = 0; i < len-1; i++){
        cv::line(rawImg, cv::Point2i(path[i*2], path[i*2+1]), cv::Point2i(path[2*(i+1)], path[2*(i+1)+1]), cv::Scalar(255, 0, 0));
    }
    cv::imwrite(img_save_path, rawImg);
    return rawImg;
};

cv::Mat utils::visualizePoints(int* path, int len, int** map, int map_width, int map_height, std::string img_save_path){
    // 颜色配置
    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Vec3i(255, 255, 255));    //可通行区域白色
    colors.push_back(cv::Vec3i(0, 0, 0)); // 不可通行区域 灰色

    // cv::Mat img(height, width, date_type, cv::Scalar())
    cv::Mat rawImg = cv::Mat::zeros(map_height, map_width, CV_8UC3);
    for (int i = 0; i < map_height; i++)
        for (int j = 0; j < map_width; j++){
            int temp = map[i][j];
            rawImg.at<cv::Vec3b>(i, j)[0] = colors[temp][0];
            rawImg.at<cv::Vec3b>(i, j)[1] = colors[temp][1];
            rawImg.at<cv::Vec3b>(i, j)[2] = colors[temp][2];
        }

    for (int i = 0; i < len; i++){
        int y = path[2*i];
        int x = path[2*i+1];
        rawImg.at<cv::Vec3b>(y, x)[0] = 255;
        rawImg.at<cv::Vec3b>(y, x)[1] = 0;
        rawImg.at<cv::Vec3b>(y, x)[2] = 0;
    }
    cv::imwrite(img_save_path, rawImg);
    return rawImg;
}

template <class T>
cv::Mat utils::visualizePath(std::vector<Node<T> > path, int** map, int map_xsize, int map_ysize, std::string img_save_path){
    // 颜色配置
    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Vec3i(128, 128, 128)); // 不可通行区域 灰色
    colors.push_back(cv::Vec3i(255, 255, 255));    //可通行区域白色

    cv::Mat rawImg = cv::Mat::zeros(map_ysize, map_xsize, CV_8UC3);
    for (int i = 0; i < int(map_ysize); i++)
        for (int j = 0; j < map_xsize; j++)
        {
            rawImg.at<cv::Vec3b>(i, j)[0] = colors[map[j][i]][0];
            rawImg.at<cv::Vec3b>(i, j)[1] = colors[map[j][i]][1];
            rawImg.at<cv::Vec3b>(i, j)[2] = colors[map[j][i]][2];
        }

    for(const auto& i: path){
        cv::circle(rawImg, cv::Point2i(i.y, i.x),3, cv::Scalar(255, 0, 0), -1);
    }
    cv::imwrite(img_save_path, rawImg);
    return rawImg;
};
