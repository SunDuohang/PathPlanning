/**
  ******************************************************************************
  * @file           : loadmap.cpp
  * @author         : SUNX
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/19
  * @last_change    : 
  ******************************************************************************
  */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>
#include "../include/loadmap.h"

int get_map_size(const char* filename, int &xSize, int &ySize)
{
    std::fstream ReadFile(filename, std::ios_base::in);
    if (!ReadFile.is_open()){
        std::cout << "File open failure" << std::endl;
        return -1;
    }
    int n = 0;
    TestEOL test{};
    std::string buf;
    std::getline(ReadFile, buf);
//    std::cout << "buf is: " << buf << std::endl;
//    std::cout << "buf length is " << buf.length() << std::endl;
    for (char i: buf)
        if (i == '\n' || i == ' ')
            n++;
    char ch = buf.back();
    if (ch != ' ' && ch != '\n')
        n++;

    std::size_t count = std::count_if(std::istreambuf_iterator<char>(ReadFile),
                                      std::istreambuf_iterator<char>(),
                                      test);

    xSize = n;
    if (test.last != '\n') {
        count++;
    }
    ySize = int(count);
    return 0;
}

void load_map(const char* filename,int** map, int xSize, int ySize){
    std::fstream read_file;
    read_file.open(filename, std::ios_base::in);
    std::string buf;
    if (!read_file.is_open()){
        std::cout << "File open failure, Maybe file is not exist!" << std::endl;
        return ;
    }
    for(int i = 0; i < ySize; i++)
    {
        for(int j = 0; j < xSize; j++){
            read_file >> map[i][j];
        }
    }
    read_file.close();
}

//void load_map(const char* filename,int* map[], int xSize, int ySize){
//    std::fstream read_file;
//    read_file.open(filename, std::ios_base::in);
//    std::string buf;
//    if (!read_file.is_open()){
//        std::cout << "File open failure, Maybe file is not exist!" << std::endl;
//        return ;
//    }
//    for(int i = 0; i < xSize; i++)
//    {
//        int *p = map[i];
//        for(int j = 0; j < ySize; j++){
//            read_file >> (*map)[j];
//        }
//    }
//    read_file.close();
//}