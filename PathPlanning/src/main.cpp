
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <regex>
#include <algorithm>
#include "../include/loadmap.h"
#include <malloc.h>
#include <ctime>
#include <chrono>
#include <cmath>
#include <memory>
#include "../include/AStar.hpp"
//#include "../include/utils.h"

void environment_grid();
void environment_grid_500x500();
void map_32x32_4();
void Random_32x32_3();
void Random_32x32_4();
void Random_64x64_3();
void Random_64x64_4();
void Random_100x100_3();
void Random_100x100_4();
void Random_500x500_3();
void empty_32x32();
void empty_500x500();
void Random_1000x1000();

template<class T1>
std::vector<Node<T1> > get_neighbors(T1** map, int map_xsize, int map_ysize, Node<T1> cur, int method);
template <class T1>
std::vector<Node<T1> > get_neighbors_new(T1** map, int map_xsize, int map_ysize, Node<T1> cur, int method);
template <class T1>
bool is_valid_neighbor(T1 x, T1 y, int map_xsize, int map_ysize, T1** map);
template <class T1>
bool is_valid_move(Node<T1> sp, Node<T1> ep, T1 **map);
template <class T1>
bool is_valid_move_new(Node<T1> sp, Node<T1> ep, T1 **map);
template <class T1>
void test();
template <class T1>
void test_is_vaild_move();
template <class T1>
bool LineOfSight(T1 **map, Node<T1> sp, Node<T1> ep);


int offsets[32][2] = {{0, -1,},{1, 0}, {0, 1}, {-1, 0},
                      {1, -1}, {1, 1}, {-1, 1}, {-1, -1},
                      {1, -2}, {2, -1},{2, 1},{1, 2},
                      {-1, 2}, {-2, 1}, {-2, -1}, {-1, -2},
                      {1, -3}, {2, -3}, {3, -2}, {3, -1},
                      {3, 1}, {3, 2}, {2, 3}, {1, 3},
                      {-1, 3}, {-2, 3}, {-3, 2}, {-3, 1},
                      {-3, -1}, {-3, -2}, {-2, -3}, {-1, -3}};

short new_offsets[32][11] = {{0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                             {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                             {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                             {-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                             {1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                             {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                             {-1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                             {-1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                             {0, -1, 1, -1, 1, -2, 0, 0, 0, 0, 3},
                             {1, 0, 1, -1, 2, -1, 0, 0, 0, 0, 3},
                             {1, 0, 1, 1, 2, 1, 0, 0, 0, 0, 3},
                             {0, 1, 1, 1, 1, 2, 0, 0, 0, 0, 3},
                             {0, 1, -1, 1, -1, 2, 0, 0, 0, 0, 3},
                             {-1, 0, -1, 1, -2, 1, 0, 0, 0, 0, 3},
                             {-1, 0, -1, -1, -2, -1, 0, 0, 0, 0, 3},
                             {0, -1, -1, -1, -1, -2, 0, 0, 0, 0, 3},
                             {0, -1, 1, -2, 1, -3, 0, 0, 0, 0, 3},
                             {0, -1, 1, -1, 1, -2, 2, -2, 2, -3, 5},
                             {1, 0, 1, -1, 2, -1, 2, -2, 3, -2, 5},
                             {1, 0, 2, -1, 3, -1, 0, 0, 0, 0, 3},
                             {1, 0, 2, 1, 3, 1, 0, 0, 0, 0, 3},
                             {1, 0, 1, 1, 2, 1, 2, 2, 3, 2, 5},
                             {0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 5},
                             {0, 1, 1, 2, 1, 3, 0, 0, 0, 0, 3},
                             {0, 1, -1, 2, -1, 3, 0, 0, 0, 0, 3},
                             {0, 1, -1, 1, -1, 2, -2, 2, -2, 3, 5},
                             {-1, 0, -1, 1, -2, 1, -2, 2, -3, 2, 5},
                             {-1, 0, -2, 1, -3, 1, 0, 0, 0, 0, 3},
                             {-1, 0, -2, -1, -3, -1, 0, 0, 0, 0, 3},
                             {-1, 0, -1, -1, -2, -1, -2, -2, -3, -2, 5},
                             {0, -1, -1, -1, -1, -2, -2, -2, -2, -3, 5},
                             {0, -1, -1, -2, -1, -3, 0, 0, 0, 0, 3}};

int main()
{
//    environment_grid();
//    environment_grid_500x500();
//    empty_32x32();
//    empty_500x500();
//    map_32x32_4();
    Random_32x32_3();
    Random_32x32_4();
    Random_64x64_3();
    Random_64x64_4();
    Random_100x100_3();
    Random_100x100_4();
//    Random_500x500_3();
//    Random_1000x1000();
//    test<int>();
//    test_is_vaild_move<int>();
    return 0;
}

void environment_grid(){
    const char* file_name = "../map_txt/environment_grid.txt";

    int** map;
    int map_xsize = 0;
    int map_ysize = 0;
    get_map_size(file_name, map_xsize, map_ysize);
    map = new int * [map_xsize];
    for(int i = 0; i < map_xsize; i++)
        map[i] = new int [map_ysize];
    load_map(file_name, map, map_xsize, map_ysize);

    Node<int> Target(1, 1);
    int start_point_list[32][2] ={{0, 30}, {2, 30}, {4, 30}, {6, 30}, {8, 30}, {10, 30},
                                {12, 30}, {14, 30}, {16, 30}, {18, 30}, {20, 30},
                                {22, 30}, {24, 30}, {26, 30}, {28, 30}, {30, 30}, {1, 31},
                                {3, 31}, {5, 31}, {7, 31}, {9, 31}, {11, 31}, {13, 31},
                                {15, 31}, {17, 31}, {19, 31}, {21, 31}, {23, 31},
                                {25, 31}, {27, 31}, {29, 31}, {31, 31},};
    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

//            test_output << "running time cost: " << duration_millsecond << " ms \t";
//            test_output << "path length: " << result.second << std::endl;
//            test_output << "method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            int pathsize = result.first.size();
            int* path = new int[pathsize*2];
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            for (int i = 0; i < pathsize; i++){
                path[2*i] = result.first[i].x;
                path[2*i+1] = result.first[i].y;
            }
            const char* plot_out = "../output/dstImg.png";
            //utils::visualizePath(path, pathsize, map, map_xsize, map_ysize, plot_out);

            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
            output_string.str("");
        }
    }

    test_output << "\n" <<record_string.str() << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    for(int i = 0; i < map_xsize; i++)
        delete []map[i];
    delete []map;

    malloc_trim(0);
}

void environment_grid_500x500(){
    const char* file_name = "../map_txt/environment_grid_500x500.txt";

    Node<int> Target(1, 1);
    int start_point_list[32][2] ={{1, 499}, {17, 499}, {31, 499}, {47, 499}, {61, 499}, {77, 499}, {91, 499}, {107, 499},
                                  {121, 498}, {137, 499}, {151, 499}, {167, 498}, {181, 499}, {197, 499}, {211, 499},
                                  {227, 499}, {231, 498}, {247, 499}, {261, 499}, {277, 499}, {291, 499}, {307, 499},
                                  {321, 499}, {337, 497}, {351, 498}, {367, 499}, {381, 499}, {397, 499},
                                  {411, 499}, {451, 499}, {475, 498}, {499, 499}};
    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

//            test_output << "running time cost: " << duration_millsecond << " ms \t";
//            test_output << "path length: " << result.second << std::endl;
//            test_output << "method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
            output_string.str("");
        }
    }

    test_output << "\n" <<record_string.str() << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    malloc_trim(0);
}

void map_32x32_4(){
    const char* file_name = "../map_txt/map_32x32_4.txt";

    Node<int> Target(1, 1);
    int start_point_list[32][2] ={{0, 30}, {2, 30}, {4, 31}, {6, 30}, {8, 30}, {10, 30},
                                  {12, 30}, {14, 31}, {16, 31}, {18, 31}, {20, 30},
                                  {22, 30}, {24, 31}, {26, 30}, {28, 30}, {30, 31}, {1, 31},
                                  {3, 31}, {5, 31}, {7, 31}, {10, 31}, {11, 31}, {13, 31},
                                  {15, 31}, {17, 31}, {18, 31}, {21, 31}, {23, 31},
                                  {25, 31}, {27, 31}, {29, 31}, {31, 31}};
    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

//            test_output << "running time cost: " << duration_millsecond << " ms \t";
//            test_output << "path length: " << result.second << std::endl;
//            test_output << "method: " << method << std::endl;

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }
    test_output << "\n" <<record_string.str() << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    malloc_trim(0);
}

void Random_32x32_3(){
    const char* file_name = "../map_txt/Random_32x32_.3.txt";

    Node<int> Target(1, 1);
    int start_point_list[32][2] = {{0, 30}, {2, 30}, {4, 29}, {6, 30}, {8, 30}, {10, 31},
                                   {12, 30}, {14, 29}, {16, 30}, {18, 30}, {20, 30},
                                   {22, 30}, {24, 31}, {26, 30}, {28, 31}, {30, 31}, {1, 30},
                                   {3, 31}, {5, 30}, {7, 31}, {9, 31}, {11, 29}, {13, 30},
                                   {15, 31}, {17, 31}, {19, 30}, {21, 31}, {23, 31},
                                   {25, 31}, {27, 31}, {29, 31}, {31, 30}};

    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

//            test_output << "running time cost: " << duration_millsecond << " ms \t";
//            test_output << "path length: " << result.second << std::endl;
//            test_output << "method: " << method << std::endl;
//            test_output << "path: " << " " ;

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }
    test_output << "\n" <<record_string.str() << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    malloc_trim(0);
}
void Random_32x32_4(){
    const char* file_name = "../map_txt/Random_32x32_.4.txt";

    Node<int> Target(1, 1);
    int start_point_list[32][2] = {{0, 30}, {1, 31}, {2, 30}, {3, 31}, {4, 29}, {5, 31}, {6, 30}, {7, 31},
                                   {8, 30}, {9, 30}, {10, 31}, {11, 27}, {12, 28}, {13, 31}, {14, 30},
                                   {15, 31}, {16, 30}, {17, 31}, {18, 30}, {19, 26}, {20, 29}, {21, 30},
                                   {22, 29}, {23, 29}, {24, 31}, {25, 31}, {26, 29}, {27, 31},
                                   {28, 30}, {29, 31}, {30, 31}, {31, 31}};

    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

//            test_output << "running time cost: " << duration_millsecond << " ms \t";
//            test_output << "path length: " << result.second << std::endl;
//            test_output << "method: " << method << std::endl;
//            test_output << "path: " << " " ;

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str()<< std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }
    test_output << "\n" << record_string.str() << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    malloc_trim(0);
}

void Random_64x64_3(){
    const char* file_name = "../map_txt/Random_64x64_.3.txt";

    Node<int> Target(1, 1);
    int start_point_list[32][2] = {{0, 62}, {2, 63}, {4, 61}, {6, 63}, {8, 62}, {10, 63}, {12, 62}, {14, 63},
                                   {16, 61}, {18, 63}, {20, 62}, {22, 62}, {24, 63}, {26, 63}, {28, 62},
                                   {30, 61}, {32, 61}, {34, 63}, {36, 63}, {38, 61}, {40, 63}, {42, 62},
                                   {44, 62}, {46, 61}, {48, 63}, {50, 63}, {52, 61}, {54, 63},
                                   {56, 63}, {58, 62}, {60, 62}, {62, 63}};

    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

//            test_output << "running time cost: " << duration_millsecond << " ms \t";
//            test_output << "path length: " << result.second << std::endl;
//            test_output << "method: " << method << std::endl;
//            test_output << "path: " << " " ;

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }
    test_output << "\n" << record_string.str() << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    malloc_trim(0);
}

void Random_64x64_4(){
    const char* file_name = "../map_txt/Random_64x64_.4.txt";

    Node<int> Target(1, 1);
    int start_point_list[32][2] = {{0, 62}, {2, 63}, {4, 61}, {6, 63}, {8, 62}, {10, 63}, {12, 62}, {14, 63},
                                   {16, 61}, {18, 63}, {20, 62}, {22, 62}, {24, 63}, {26, 63}, {28, 62},
                                   {30, 61}, {32, 61}, {34, 63}, {36, 63}, {38, 61}, {40, 63}, {42, 62},
                                   {44, 62}, {46, 61}, {48, 63}, {50, 63}, {52, 61}, {54, 63},
                                   {56, 63}, {58, 62}, {60, 62}, {62, 63}};

    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

            test_output << "running time cost: " << duration_millsecond << " ms \t";
            test_output << "path length: " << result.second << std::endl;
            test_output << "method: " << method << std::endl;
            test_output << "path: " << " " ;

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() <<std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }

    test_output << "\n" << record_string.str() << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    malloc_trim(0);
}

void Random_100x100_3(){
    const char* file_name = "../map_txt/Random_100x100_.3.txt";

    Node<int> Target(1, 1);
    int start_point_list[32][2] = {{0, 99}, {3, 99}, {6, 99}, {9, 99}, {12, 98}, {15, 98}, {18, 99}, {21, 99},
                                   {24, 99}, {27, 98}, {30, 99}, {33, 99}, {36, 99}, {39, 99}, {42, 98},
                                   {45, 99}, {48, 99}, {51, 99}, {54, 99}, {57, 98}, {60, 99}, {64, 98},
                                   {67, 99}, {70, 98}, {73, 98}, {77, 99}, {81, 99}, {84, 99},
                                   {87, 99}, {90, 99}, {93, 98}, {96, 99}};

    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

//            test_output << "running time cost: " << duration_millsecond << " ms \t";
//            test_output << "path length: " << result.second << std::endl;
//            test_output << "method: " << method << std::endl;
//            test_output << "path: " << " " ;

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }

    test_output << "\n" << record_string.str() << std::endl;
    test_output << "\n" << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    malloc_trim(0);
}

void Random_100x100_4(){
    const char* file_name = "../map_txt/Random_100x100_.4.txt";

    Node<int> Target(1, 1);
    int start_point_list[32][2] = {{0, 99}, {3, 99}, {6, 99}, {9, 99}, {12, 98}, {15, 99}, {18, 99}, {21, 99},
                                   {24, 99}, {27, 98}, {30, 99}, {33, 99}, {36, 98}, {39, 98}, {42, 97},
                                   {45, 99}, {48, 99}, {51, 99}, {54, 98}, {57, 99}, {60, 98}, {64, 98},
                                   {67, 99}, {70, 98}, {73, 98}, {77, 99}, {81, 99}, {84, 99},
                                   {87, 97}, {90, 97}, {93, 98}, {96, 99}};

    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

//            test_output << "running time cost: " << duration_millsecond << " ms \t";
//            test_output << "path length: " << result.second << std::endl;
//            test_output << "method: " << method << std::endl;
//            test_output << "path: " << " " ;

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }

    test_output << "\n" << record_string.str() << std::endl;
    test_output << "\n" << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    malloc_trim(0);
}

void Random_500x500_3(){
    const char* file_name = "../map_txt/Random_500x500_.2.txt";

    Node<int> Target(1, 1);
    int start_point_list[32][2] = {{0, 498}, {15, 499}, {30, 499}, {45, 499}, {62, 499}, {75, 499}, {90, 499}, {105, 499},
                                   {120, 498}, {135, 499}, {150, 499}, {165, 498}, {180, 499}, {195, 499}, {210, 499},
                                   {225, 499}, {240, 498}, {255, 499}, {270, 499}, {285, 499}, {300, 499}, {315, 499},
                                   {330, 499}, {345, 497}, {360, 498}, {375, 499}, {390, 499}, {405, 499},
                                   {420, 499}, {435, 499}, {475, 498}, {499, 499}};

    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

//            test_output << "running time cost: " << duration_millsecond << " ms \t";
//            test_output << "path length: " << result.second << std::endl;
//            test_output << "method: " << method << std::endl;
//            test_output << "path: " << " " ;

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            std::reverse(result.first.begin(), result.first.end());

            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }
    test_output << "\n" << record_string.str() << std::endl;
    test_output << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    malloc_trim(0);
}

void empty_32x32(){
    const char* file_name = "../map_txt/empty_32x32.txt";

    Node<int> Target(0, 0);
    int start_point_list[32][2] = {{0, 31}, {1, 31}, {2, 31},{3, 31}, {4, 31},{5, 31}, {6, 31},{7, 31},
                                   {8, 31}, {9, 31}, {10, 31}, {11, 31}, {12, 31}, {13, 31}, {14, 31},
                                   {15, 31}, {16, 31},{17, 31}, {18, 31}, {19, 31}, {20, 31},{21, 31},
                                   {22, 31}, {23, 31}, {24, 31}, {25, 31}, {26, 31}, {27, 31},
                                   {28, 31}, {29, 31}, {30, 31}, {31, 31}};

    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

            test_output << "running time cost: " << duration_millsecond << " ms \t";
            test_output << "path length: " << result.second << std::endl;
            test_output << "method: " << method << std::endl;
            test_output << "path: " << " " ;

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";

            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }

    test_output << "\n" << record_string.str() << std::endl;

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    test_output.close();
    malloc_trim(0);
}

void empty_500x500(){
    const char* file_name = "../map_txt/empty_500x500.txt";

    Node<int> Target(0, 0);
    int start_point_list[32][2] = {{0, 499}, {15, 499}, {30, 499}, {45, 499}, {60, 499}, {75, 499}, {90, 499}, {105, 499},
                                   {120, 498}, {135, 499}, {150, 499}, {165, 498}, {180, 499}, {195, 499}, {210, 499},
                                   {225, 499}, {240, 498}, {255, 499}, {270, 499}, {285, 499}, {300, 499}, {315, 499},
                                   {330, 499}, {345, 497}, {360, 498}, {375, 499}, {390, 499}, {405, 499},
                                   {420, 499}, {435, 499}, {475, 498}, {499, 499}};

    int methods[4] = {4, 8, 16, 32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

            test_output << "running time cost: " << duration_millsecond << " ms \t";
            test_output << "path length: " << result.second << std::endl;
            test_output << "method: " << method << std::endl;
            test_output << "path: " << " " ;

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";

            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }

    test_output << "\n" << record_string.str() << std::endl;

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    test_output.close();
    malloc_trim(0);
}

void Random_1000x1000(){
    const char* file_name = "../map_txt/Random_1000x1000_.4.txt";

    Node<int> Target(1, 1);
    int start_point_list[32][2] = {{0, 998}, {34, 999}, {64, 999}, {94, 999}, {123, 999}, {161, 999}, {192, 999}, {224, 999},
                                   {255, 999}, {286, 999}, {317, 999}, {349, 999}, {380, 999}, {412, 999}, {445, 999},
                                   {476, 999}, {508, 999}, {540, 999}, {572, 999}, {604, 999}, {636, 999}, {668, 999},
                                   {700, 998}, {732, 999}, {764, 998}, {796, 999}, {828, 996}, {860, 999},
                                   {892, 998}, {924, 999}, {956, 999}, {999, 999}};

    int methods[3] = { 8, 16, 32};
//    int methods[1] = {32};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << "year" << p->tm_mon+1 << "month " << p->tm_mday <<
               "day  " << p->tm_hour << ":" << p->tm_min <<":" << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    Astar<int, Record_Item<int>> astar (file_name);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node<int> Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node<int> >, double> result = astar.run(file_name, Start, Target, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";

            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node<int> > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }

    test_output << "\n" <<record_string.str() << std::endl;

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    test_output.close();
    malloc_trim(0);
}

template <class T1>
void test(){
    T1** map;
    int map_xsize = 0;
    int map_ysize = 0;
    char* filename = "../map_txt/Random_32x32_.3.txt";
    get_map_size(filename, map_xsize, map_ysize);
    map = new T1 * [map_xsize];
    for(int i = 0; i < map_xsize; i++)
        map[i] = new T1 [map_ysize];
    load_map(filename, map, map_xsize, map_ysize);
    std::string out_filename = R"(../output/whichisfast_log.txt)";
    std::fstream out_file(out_filename, std::ios_base::app | std::ios_base::out);

    int start_xy_list[10][2] = {{4, 7},
                                {5, 7},
                                {6,7},
                                {7, 7},
                                {7, 9},
                                {7, 10},
                                {7, 12},
                                {10, 8},
                                {10, 7},
                                {10, 6}};


    int count = 200;
    int method = 32;
    while (count){
        for(int i = 0; i < 10; i++){
            Node<int> cur(start_xy_list[i][0], start_xy_list[i][1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            get_neighbors<T1>(map, map_xsize, map_ysize, cur, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();
            std::stringstream output_string;
            output_string <<  "get_neighbors cost: " << duration_millsecond << " ms \t"  << " method: " << method << "\t";
            before_time = std::chrono::high_resolution_clock::now();
            get_neighbors_new<T1>(map, map_xsize, map_ysize, cur, method);
            after_time = std::chrono::high_resolution_clock::now();
            duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();
            output_string <<  "get_neighbors_new cost: " << duration_millsecond << " ms \t"  << " method: " << method << "\n";
            out_file << output_string.str();

            output_string.clear();
            output_string.str("");
        }
        count --;
    }

    out_file.close();

    for(int i = 0; i < map_xsize; i++)
        delete []map[i];
    delete []map;
}

template <class T1>
void test_is_vaild_move(){
    T1** map;
    int map_xsize = 0;
    int map_ysize = 0;
    char* filename = "../map_txt/Random_32x32_.3.txt";
    get_map_size(filename, map_xsize, map_ysize);
    map = new T1 * [map_xsize];
    for(int i = 0; i < map_xsize; i++)
        map[i] = new T1 [map_ysize];
    load_map(filename, map, map_xsize, map_ysize);
    std::string out_filename = R"(../output/whichisright_log.txt)";
    std::fstream out_file(out_filename, std::ios_base::app | std::ios_base::out);

    int start_xy_list[10][2] = {{4, 7},
                                {5, 7},
                                {6,7},
                                {7, 7},
                                {7, 9},
                                {7, 10},
                                {7, 12},
                                {10, 8},
                                {10, 7},
                                {10, 6}};


    int count = 1;
    int method = 32;
    while (count){
        for(int i = 0; i < 10; i++){
            Node<int> cur(start_xy_list[i][0], start_xy_list[i][1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            get_neighbors<T1>(map, map_xsize, map_ysize, cur, method);
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();
            std::stringstream output_string;
            output_string <<  "get_neighbors cost: " << duration_millsecond << " ms \t"  << " method: " << method << "\t";
//            before_time = std::chrono::high_resolution_clock::now();
//            get_neighbors_new<T1>(map, map_xsize, map_ysize, cur, method);
//            after_time = std::chrono::high_resolution_clock::now();
//            duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();
//            output_string <<  "get_neighbors_new cost: " << duration_millsecond << " ms \t"  << " method: " << method << "\n";
            out_file << output_string.str() << std::endl;
            output_string.clear();
            output_string.str("");
        }
        count --;
    }

    out_file.close();

    for(int i = 0; i < map_xsize; i++)
        delete []map[i];
    delete []map;
}


template <class T1>
std::vector<Node<T1> > get_neighbors(T1** map, int map_xsize, int map_ysize, Node<T1> cur, int method){
//    int offsets[32][2] = {{0, -1,},{1, 0}, {0, 1}, {-1, 0},
//                          {1, -1}, {1, 1}, {-1, 1}, {-1, -1},
//                          {1, -2}, {2, -1},{2, 1},{1, 2},
//                          {-1, 2}, {-2, 1}, {-2, -1}, {-1, -2},
//                          {1, -3}, {2, -3}, {3, -2}, {3, -1},
//                          {3, 1}, {3, 2}, {2, 3}, {1, 3},
//                          {-1, 3}, {-2, 3}, {-3, 2}, {-3, 1},
//                          {-3, -1}, {-3, -2}, {-2, -3}, {-1, -3}};
    std::vector<Node<T1> > neighbors;
    T1 cur_x = cur.x;
    T1 cur_y = cur.y;
    T1 neighbor_x = (T1)0;
    T1 neighbor_y = (T1)0;
    if (method == 4 || method == 8){
        for (int i = 0; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor<int>(neighbor_x, neighbor_y, map_xsize, map_ysize, map))
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
        }
    }
    else if(method == 16 || method == 32){
        for (int i = 0; i < 8; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map))
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
        }
        for (int i = 8; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map) && LineOfSight(map, cur, Node<int>(neighbor_x, neighbor_y)))
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
        }
    }

    std::string out_filename = R"(../output/is_valid_move_LineOfSight.txt)";
    std::fstream outfile(out_filename, std::ios_base::app | std::ios_base::out);
    std::stringstream out_string;
    out_string << "neighbors are: " ;
    for (Node<T1> node: neighbors){
        out_string << "[" << node.x << ", " << node.y << "], ";
    }
    out_string << "\n";
    outfile << out_string.str() ;
    out_string.clear();
    out_string.str("");
    outfile.close();

    return neighbors;
};

template <class T1>
std::vector<Node<T1> > get_neighbors_new(T1** map, int map_xsize, int map_ysize, Node<T1> cur, int method){
    std::vector<Node<T1> > neighbors;
    T1 cur_x = cur.x;
    T1 cur_y = cur.y;
    T1 neighbor_x = (T1)0;
    T1 neighbor_y = (T1)0;
    int flag = 1;
    for(int i = 0; i < method; i++){
        int size = new_offsets[i][10];
        flag = 1;
        for (int j = 0; j < size; j++){
            neighbor_x = cur_x + new_offsets[i][2 * j];
            neighbor_y = cur_y + new_offsets[i][2 * j + 1];
            if (!is_valid_neighbor<T1>(neighbor_x, neighbor_y, map_xsize, map_ysize, map)){
                flag = 0;
                break;
            }
        }
        if(flag)
            neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
    }

    std::string out_filename = R"(../output/get_neighbors_new.txt)";
    std::fstream outfile(out_filename, std::ios_base::app | std::ios_base::out);
    std::stringstream out_string;
    out_string << "neighbors are: " ;
    for (Node<T1> node: neighbors){
        out_string << "[" << node.x << ", " << node.y << "], ";
    }
    out_string << "\n";
    outfile << out_string.str() ;
    out_string.clear();
    out_string.str("");
    outfile.close();

    return neighbors;
}

template <class T1>
bool is_valid_neighbor(T1 x, T1 y, int map_xsize, int map_ysize, T1** map) {
    if( x >= 0 && x < map_xsize && y >= 0 && y < map_ysize && map[x][y] == 0)
        return true;
    return false;
}


template <class T1>
bool is_valid_move(Node<T1> sp, Node<T1> ep, T1 **map) {
    double slope = 0.0;
    int x1 = (int)sp.x;
    int y1 = (int)sp.y;
    int x2 = (int)ep.x;
    int y2 = (int)ep.y;
    if (x1 == x2){
        if(y1 > y2){
            int tmp = y1;
            y1 = y2;
            y2 = tmp;
        }
        for (int i = y1; i < y2; i++){
            if (map[x1][i] == 1) // obstacle
                return false;
        }
    }
    else if(y1 == y2){
        if(x1 > x2){
            int tmp = x1;
            x1 = x2;
            x2 = tmp;
        }
        for(int i = x1; i < x2; i++){
            if (map[i][y1] == 1)    //obstacle
                return false;
        }
    }
    else if(sp.x > ep.x){
        x1 = ep.x;
        x2 = sp.x;
        y1 = ep.y;
        y2 = sp.y;
        slope = 1.0 * (y2 - y1) / (x2 - x1);
    }
    else{
        slope = 1.0 * (y2 - y1) / (x2 - x1);
    }
    if (fabs((slope - 0.0)) > 1e-2){
        for (double xi = x1; xi < x2; ){
            int tmp_x = round(xi);
            int tmp_y = round(slope*(xi - x1) + y1);
            if (map[tmp_x][tmp_y] == 1)    //obstacle
                return false;
            xi = xi + 0.1;
        }
    }
    return true;
}

template <class T1>
bool is_valid_move_new(Node<T1> sp, Node<T1> ep, T1 **map) {
    T1 x1 = sp.x;
    T1 x2 = ep.x;
    T1 y1 = sp.y;
    T1 y2 = ep.y;
    double dx = x2 - x1;
    double dy = y2 - y1;
    int sx = 1;
    int sy = 1;
    std::cout << "start: " << sp.x << ", " << sp.y << " target: " << ep.x << ", " << ep.y << std::endl;
    if (dy < 0.0){
        dy = -dy;
        sy = -1;
        y1 += sx;
    }
    if (dx < 0.0){
        dx = -dx;
        sx = -1;
        x1 += sx;
    }
    if (dy == 0.0){
        while (x1 != x2){
            std::cout << "map[" << x1 << ", " << y1 << "] " << std::endl;
            if(map[x1][y1])
                return false;
            x1 += sx;
        }
    }
    else if(dx == 0.0){
        while(y1 != y2){
            std::cout << "map[" << x1 << ", " << y1 << "] " << std::endl;
            if(map[x1][y1])
                return false;
            y1 += sy;
        }
    }
    else{
        double slope = dy /dx /** sx * sy*/;
        std::cout << "slope " << slope << std::endl;
        int slope_ceil = ceil(slope) * sy;
        int y_ceil = y1 + slope_ceil;
        while(x1 != x2){
            std::cout << "y_ceil " << y_ceil << std::endl;
            while(y1 != y_ceil){
                std::cout << "map[" << x1 << ", " << y1 << "] " << std::endl;
                if(map[x1][y1])
                    return false;
                y1 += sy;
            }
            y1 -= sy;
            x1 += sx;
            y_ceil = y1 + slope_ceil;
        }
    }
/*    if (dx >= dy){
        {
            x1 += sx;
            double slope = dy / dx;
            int slope_ceil = ceil(slope) ;
            while(x1 != x2){
                T1 y_ceil = y1 + slope_ceil;
                while(y1 != y_ceil){
                    std::cout << "map[" << x1 << ", " << y1 << "] " << std::endl;
                    if (map[x1][y1])
                        return false;
                    y1 += sy;
                }
                y1 -= sy;
                x1 += sx;
            }
        }
    } else{
        {
            y1 += sy;
            double slope = dx / dy;
            int slope_ceil = ceil(slope);
            while(y1 != y2){
                T1 x_ceil = x1 + slope_ceil;
                while(x1 != x_ceil){
                    std::cout << "map[" << x1 << ", " << y1 << "] " << std::endl;
                    if (map[x1][y1])
                        return false;
                    x1 += sx;
                }
                x1 -= sx;
                y1 += sy;
            }
        }
    }*/
    return true;
}

template <class T1>
bool LineOfSight(T1 **map, Node<T1> sp, Node<T1> ep) {
    T1 y0 = sp.x;
    T1 y1 = ep.x;
    T1 x0 = sp.y;
    T1 x1 = ep.y;
    double dy = y1 - y0;
    double dx = x1 - x0;
    int sx = 1;
    int sy = 1;
    std::cout << "start: " << sp.x << ", " << sp.y << " target: " << ep.x << ", " << ep.y << std::endl;

    if (dx < 0.0){
        dx = -dx;
        sx = -1;
        x0 += sx;
    }
    if (dy < 0.0){
        dy = -dy;
        sy = -1;
        y0 += sy;
    }
    if (dx == 0.0){
        while (y0 != y1){
            std::cout << "map[" << y0 << ", " << x0 << "] " << std::endl;
            if(map[y0][x0])
                return false;
            y0 += sy;
        }
    }
    else if(dy == 0.0){
        while(x0 != x1){
            std::cout << "map[" << y0 << ", " << x0 << "] " << std::endl;
            if(map[y0][x0])
                return false;
            x0 += sx;
        }
    }
    else{
        double slope = dx /dy /** sx * sy*/;
        std::cout << "slope " << slope << std::endl;
        int slope_ceil = ceil(slope);
//        if (slope < 0.0)
//            slope_ceil = floor(slope);
        int y_ceil = x0 + slope_ceil*sy;
        while(x0 != x1){
            std::cout << "y_ceil " << y_ceil << std::endl;
            while(y0 != y_ceil){
                std::cout << "map[" << y0 << ", " << x0 << "] " << std::endl;
                if(map[y0][x0])
                    return false;
                y0 += sy;
            }
            y0 -= sy;
            y_ceil = y0 + slope_ceil;
            x0 += sx;
        }
    }
/*    if (dx >= dy){
        {
            x1 += sx;
            double slope = dy / dx;
            int slope_ceil = ceil(slope) ;
            while(x1 != x2){
                T1 y_ceil = y1 + slope_ceil;
                while(y1 != y_ceil){
                    std::cout << "map[" << x1 << ", " << y1 << "] " << std::endl;
                    if (map[x1][y1])
                        return false;
                    y1 += sy;
                }
                y1 -= sy;
                x1 += sx;
            }
        }
    } else{
        {
            y1 += sy;
            double slope = dx / dy;
            int slope_ceil = ceil(slope);
            while(y1 != y2){
                T1 x_ceil = x1 + slope_ceil;
                while(x1 != x_ceil){
                    std::cout << "map[" << x1 << ", " << y1 << "] " << std::endl;
                    if (map[x1][y1])
                        return false;
                    x1 += sx;
                }
                x1 -= sx;
                y1 += sy;
            }
        }
    }*/
    return true;
}