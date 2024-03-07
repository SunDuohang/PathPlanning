/**
  ******************************************************************************
  * @file           : DataLoad.cpp
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-4
  ******************************************************************************
  */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>
#include <regex>
#include "../include/DataLoad.h"

/**
 * @brief
 * @param filename
 * @param width
 * @param height
 * @return
 */
bool get_map_size(const char *filename, int &width, int &height){
    std::fstream ReadFile(filename, std::ios_base::in);
    if (!ReadFile.is_open()){
        std::cout << "File open failure" << std::endl;
        return false;
    }
    int n = 0;
    TestEOL test{};
    std::string buf;
    std::getline(ReadFile, buf);
    for (char i: buf)
        if (i == '\n' || i == ' ')
            n++;
    char ch = buf.back();
    if (ch != ' ' && ch != '\n')
        n++;

    std::size_t count = std::count_if(std::istreambuf_iterator<char>(ReadFile),
                                      std::istreambuf_iterator<char>(),
                                      test);

    if (test.last != '\n') {
        count++;
    }
    width = n;
    height = int(count);
    return true;
};

/**
 * @param filename
 * @param map 2d matrix map[height][width]
 * @param width
 * @param height
 */
void load_map(const char *filename, int **map, int width, int height){
    std::fstream read_file;
    read_file.open(filename, std::ios_base::in);
    if (!read_file.is_open()){
        std::cout << "File open failure, Maybe file is not exist!" << std::endl;
        return ;
    }
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++){
            read_file >> map[i][j];
        }
    }
    read_file.close();
};
/**
 * @brief 读取txt文件到vector
 * @param filename
 * @param map
 */
void load_data(const char *filename, std::vector<std::vector<int> > &data){
    std::vector<int> temp_line;
    std::fstream read_file;
    read_file.open(filename, std::ios_base::in);

    if (!read_file.is_open()){
        std::cout << "File open failure, Maybe file is not exist!" << std::endl;
        return ;
    }
    std::regex pat_regex("[[:digit:]]+");  //匹配原则，这里代表一个或多个数字

    std::string line;
    while (getline(read_file, line)){
        for (std::sregex_iterator it(line.begin(), line.end(), pat_regex), end_it; it != end_it; ++it) {  //表达式匹配，匹配一行中所有满足条件的字符
            temp_line.push_back(stoi(it->str()));  //将数据转化为int型并存入一维vector中
        }
        data.push_back(temp_line);  //保存所有数据

        temp_line.clear();
    };

    if (data.empty())
        std::cout << "Vector data is empty. Something is wrong!" << std::endl;

    temp_line.shrink_to_fit();
};

