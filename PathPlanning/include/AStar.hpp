//
// Created by sunx on 23-12-22.
//

#ifndef PATHPLANNING_ASTAR_HPP
#define PATHPLANNING_ASTAR_HPP

#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <cmath>
#include <cstring>
#include <memory>
#include <malloc.h>
#include "./DataLoad.h"
#include "./List.hpp"
#include "./DataStructure.hpp"

template <class T1, class T2>
class Astar{
private:
//    T map {std::make_unique<T **>()};
    T1** self_map;
    int map_Xsize{};
    int map_Ysize{};
    std::string s;

    int offsets[32][2] = {{0, -1,},{1, 0}, {0, 1}, {-1, 0},
                          {1, -1}, {1, 1}, {-1, 1}, {-1, -1},
                          {1, -2}, {2, -1},{2, 1},{1, 2},
                          {-1, 2}, {-2, 1}, {-2, -1}, {-1, -2},
                          {1, -3}, {2, -3}, {3, -2}, {3, -1},
                          {3, 1}, {3, 2}, {2, 3}, {1, 3},
                          {-1, 3}, {-2, 3}, {-3, 2}, {-3, 1},
                          {-3, -1}, {-3, -2}, {-2, -3}, {-1, -3}};

    short new_offsets[32][9] = {{0, -1, 0, 0, 0, 0, 0, 0, 1},
                                {1, 0, 0, 0, 0, 0, 0, 0, 1},
                                {0, 1, 0, 0, 0, 0, 0, 0, 1},
                                {-1, 0, 0, 0, 0, 0, 0, 0, 1},
                                {1, -1, 0, 0, 0, 0, 0, 0, 1},
                                {1, 1, 0, 0, 0, 0, 0, 0, 1},
                                {-1, 1, 0, 0, 0, 0, 0, 0, 1},
                                {-1, -1, 0, 0, 0, 0, 0, 0, 1},
                                {0, -1, 1, -1, 1, -2, 0, 0, 3},
                                {1, 0, 1, -1, 2, -1, 0, 0, 3},
                                {1, 0, 1, 1, 2, 1, 0, 0, 3},
                                {0, 1, 1, 1, 1, 2, 0, 0, 3},
                                {0, 1, -1, 1, -1, 2, 0, 0, 3},
                                {-1, 0, -1, 1, -2, 1, 0, 0, 3},
                                {-1, 0, -1, -1, -2, -1, 0, 0, 3},
                                {0, -1, -1, -1, -1, -2, 0, 0, 3},
                                {0, -1, 1, -2, 1, -3, 0, 0, 3},
                                {0, -1, 1, -1, 1, -2, 2, -2, 4},
                                {1, 0, 1, -1, 2, -1, 2, -2, 4},
                                {1, 0, 2, -1, 3, -1, 0, 0, 3},
                                {1, 0, 2, 1, 3, 1, 0, 0, 3},
                                {1, 0, 1, 1, 2, 1, 2, 2, 4},
                                {0, 1, 1, 1, 1, 2, 2, 2, 4},
                                {0, 1, 1, 2, 1, 3, 0, 0, 3},
                                {0, 1, -1, 2, -1, 3, 0, 0, 3},
                                {0, 1, -1, 1, -1, 2, -2, 2, 4},
                                {-1, 0, -1, 1, -2, 1, -2, 2, 4},
                                {-1, 0, -2, 1, -3, 1, 0, 0, 3},
                                {-1, 0, -2, -1, -3, -1, 0, 0, 3},
                                {-1, 0, -1, -1, -2, -1, -2, -2, 4},
                                {0, -1, -1, -1, -1, -2, -2, -2, 4},
                                {0, -1, -1, -2, -1, -3, 0, 0, 3}};

//    short new_offsets[32][11] = {{0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//                                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//                                {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//                                {-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//                                {1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//                                {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//                                {-1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//                                {-1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//                                {0, -1, 1, -1, 1, -2, 0, 0, 0, 0, 3},
//                                {1, 0, 1, -1, 2, -1, 0, 0, 0, 0, 3},
//                                {1, 0, 1, 1, 2, 1, 0, 0, 0, 0, 3},
//                                {0, 1, 1, 1, 1, 2, 0, 0, 0, 0, 3},
//                                {0, 1, -1, 1, -1, 2, 0, 0, 0, 0, 3},
//                                {-1, 0, -1, 1, -2, 1, 0, 0, 0, 0, 3},
//                                {-1, 0, -1, -1, -2, -1, 0, 0, 0, 0, 3},
//                                {0, -1, -1, -1, -1, -2, 0, 0, 0, 0, 3},
//                                {0, -1, 1, -2, 1, -3, 0, 0, 0, 0, 3},
//                                {0, -1, 1, -1, 1, -2, 2, -2, 2, -3, 5},
//                                {1, 0, 1, -1, 2, -1, 2, -2, 3, -2, 5},
//                                {1, 0, 2, -1, 3, -1, 0, 0, 0, 0, 3},
//                                {1, 0, 2, 1, 3, 1, 0, 0, 0, 0, 3},
//                                {1, 0, 1, 1, 2, 1, 2, 2, 3, 2, 5},
//                                {0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 5},
//                                {0, 1, 1, 2, 1, 3, 0, 0, 0, 0, 3},
//                                {0, 1, -1, 2, -1, 3, 0, 0, 0, 0, 3},
//                                {0, 1, -1, 1, -1, 2, -2, 2, -2, 3, 5},
//                                {-1, 0, -1, 1, -2, 1, -2, 2, -3, 2, 5},
//                                {-1, 0, -2, 1, -3, 1, 0, 0, 0, 0, 3},
//                                {-1, 0, -2, -1, -3, -1, 0, 0, 0, 0, 3},
//                                {-1, 0, -1, -1, -2, -1, -2, -2, -3, -2, 5},
//                                {0, -1, -1, -1, -1, -2, -2, -2, -2, -3, 5},
//                                {0, -1, -1, -2, -1, -3, 0, 0, 0, 0, 3}};


public:
    explicit Astar(const char* filename);
    ~Astar();
    //bool run(Node<T1> start_point, Node<T1> target_point, int method=16);
    std::pair<std::vector<Node<T1> >, double> run(const char* filename,Node<T1> start_point, Node<T1> target_point, int method=16);
    std::vector<Node<T1> > get_neighbors(T1** map, int map_xsize, int map_ysize, Node<T1> cur,int method=16);
    std::vector<Node<T1> > get_neighbors_new(T1** map, int map_xsize, int map_ysize, Node<T1> cur,int method=16);
    bool is_valid_neighbor(T1 x, T1 y, int map_xsize, int map_ysize, T1** map);
//    bool is_valid_move_new(T1 **map, Node<T1> sp, Node<T1> ep);
    bool is_valid_move(T1 **map, Node<T1> sp, Node<T1> ep, int map_xsize, int map_ysize);
    bool is_valid_move_new(T1 **map, Node<T1> cur, int map_xsize, int map_ysize, int i);
    double heuristic(T1 x1_cor, T1 y1_cor, T1 x2_cor, T1 y2_cor);
    double distance(T1 x1_cor, T1 y1_cor, T1 x2_cor, T1 y2_cor);
    std::vector<Node<T1> > get_lines(Node<T1> sp, Node<T1> ep);
    bool is_in_line(std::vector<Node<T1> > line, Node<T1>p);
    std::pair<std::vector<Node<T1> >, double> construct_path(std::vector<Record_Item<T1> > close_list, Node<T1> cur_node);
    int is_in_list(std::vector<Record_Item<T1> > close_list, Node<T1> cur_node);
    bool is_in_open_list(std::vector<Open_list_Item<T1> > open_list, Node<T1> cur_node);
};

template <class T1, class T2>
Astar<T1, T2>::Astar(const char* filename){

    get_map_size(filename, map_Xsize, map_Ysize);
    self_map = new T1* [map_Xsize];
    for(int i = 0; i < map_Xsize; i++)
        self_map[i] = new T1 [map_Ysize];
    load_map(filename, self_map, map_Xsize, map_Ysize);
}

template <class T1, class T2>
Astar<T1, T2>::~Astar(){
    for(int i = 0; i < map_Xsize; i++){
        delete []self_map[i];
    }
    delete []self_map;
    malloc_trim(0);
}

//template <class T1, class T2>
//bool Astar<T1, T2>::run(Node<T1> start_point, Node<T1> target_point, int method){
//    std::vector<Record_Item<T1>> open_list;
//    int map_xsize = 0;
//    int map_ysize = 0;
//
//}

template <class T1, class T2>
std::pair<std::vector<Node<T1> >, double> Astar<T1, T2>::run(const char* filename,Node<T1>start_point, Node<T1> target_point, int method){
    T1** map;
    int map_xsize = 0;
    int map_ysize = 0;
    get_map_size(filename, map_xsize, map_ysize);
    map = new T1 * [map_xsize];
    for(int i = 0; i < map_xsize; i++)
        map[i] = new T1 [map_ysize];
    load_map(filename, map, map_xsize, map_ysize);
    std::vector<Record_Item<T1> > close_list;
    std::vector<Record_Item<T1> > open_list;
//    Node<T1> self_start_point = start_point;
//    Node<T1> self_target_point = target_point;

//    // 该部分可以删除
//    std::fstream openlist_output(R"(../output/open_list.txt)", std::ios_base::out | std::ios_base::app);
//    std::fstream closelist_output(R"(../output/close_list.txt)", std::ios_base::out | std::ios_base::app);
//    std::stringstream openlist_out_string;
//    std::stringstream closelist_out_string;
//    openlist_out_string << "\n" << filename ;
//    openlist_output << openlist_out_string.str() << std::endl;
//    closelist_output << openlist_out_string.str() << std::endl;
//    openlist_out_string.clear();
//    openlist_out_string.str("");

    // 该部分不可以删除
    double g_value = 0.0;
    double h_value = heuristic(start_point.x, start_point.y, target_point.x, target_point.y);
    double f_value = g_value + h_value;
    Record_Item<T1> start_info(start_point, g_value, f_value, start_point, 0);
    open_list.push_back(start_info);

//    //该部分可以删除
//    openlist_out_string << "openlist: \n" << "[" << start_info.self_cur_cor.x << ", " << start_info.self_cur_cor.y << "], " <<
//    start_info.self_g_value << ", " << start_info.self_f_value << ", [" << start_info.self_parent_cor.x << ", " << start_info.self_parent_cor.y << "]; \n";
//    openlist_output << openlist_out_string.str() ;
//    openlist_out_string.clear();
//    openlist_out_string.str("");

    // 创建为最大堆
    std::make_heap(open_list.begin(), open_list.end(), greater1<T1>());
    while(!open_list.empty()){
        Record_Item<T1> open_item = open_list[0];
        std::pop_heap(open_list.begin(), open_list.end(), greater1<T1>());
        open_list.pop_back();
        Node<T1> cur_node = open_item.self_cur_cor;
        Node<T1> parent_node = open_item.self_parent_cor;
//        g_value = open_item.self_g_value;
//        h_value = heuristic(cur_node.x, cur_node.y, target_point.x, target_point.y);
//        f_value = g_value + h_value;
        int parentnode_index = is_in_list(close_list, parent_node);
        if (parentnode_index == -1){
            Record_Item<T1> close_item(open_item, 0);
            close_list.push_back(close_item);

//            //该部分可以删除
//            closelist_out_string << "closelist: \n" << "[" << close_item.self_cur_cor.x << ", " << close_item.self_cur_cor.y << "], " <<
//                                 close_item.self_g_value << ", " << close_item.self_f_value << ", [" << close_item.self_parent_cor.x << ", " << close_item.self_parent_cor.y << "]; \n" ;
//            closelist_output << closelist_out_string.str() ;
//            closelist_out_string.clear();
//            closelist_out_string.str("");
        }
        else{
            Record_Item<T1> close_item(open_item, parentnode_index);
            close_list.push_back(close_item);

//            //该部分可以删除
//            closelist_out_string << "closelist: \n" << "[" << close_item.self_cur_cor.x << ", " << close_item.self_cur_cor.y << "], " <<
//                                close_item.self_g_value << ", " << close_item.self_f_value << ", [" << close_item.self_parent_cor.x << ", " << close_item.self_parent_cor.y << "]; \n" ;
//            closelist_output << closelist_out_string.str() ;
//            closelist_out_string.clear();
//            closelist_out_string.str("");
        }

        if (target_point.x == cur_node.x && target_point.y == cur_node.y){
            auto result = construct_path(close_list, cur_node);

            //该部分可以删除
            std::string output_filename = R"(../output/record_log.txt)";
            std::fstream record_output(output_filename, std::ios_base::out | std::ios_base::app);
            std::stringstream out_string;
            out_string << filename << "\t";
            out_string << "strat_point-target_point: [" << start_point.x << ", " << start_point.y << "]-[" << target_point.x << ", " << target_point.y << "]" << "\t";
            out_string << "open_list size: " << open_list.size() << "\t close_list size: " << close_list.size() << "\t" ;
            out_string << "path_size: " << result.first.size() << "\t";
            record_output << out_string.str() << std::endl;
            out_string.clear();
            out_string.str("");
            record_output.close();
//            openlist_output.close();

            for(int i = 0; i < map_xsize; i++)
                delete []map[i];
            delete []map;

            open_list.clear();
            open_list.shrink_to_fit();
            close_list.clear();
            close_list.shrink_to_fit();
            return result;
        }

        std::vector<Node<T1> > neighbors = get_neighbors_new(map, map_xsize, map_ysize, cur_node, method);
        for(Node<T1> node: neighbors){
            if (is_in_list(close_list, node) != -1)
                continue;
            else{
                int open_list_index = is_in_list(open_list, node);
                if (open_list_index != -1){
                    double cur_g_value = open_item.self_g_value + distance(cur_node.x, cur_node.y, node.x, node.y);
                    if (cur_g_value < open_list[open_list_index].self_g_value){
                        double cur_h_value = heuristic(node.x, node.y, target_point.x, target_point.y);
                        open_list.erase(open_list.begin()+open_list_index);
                        Record_Item<T1> node_info(node, cur_g_value,  (cur_g_value + cur_h_value), cur_node, open_list.size());
                        open_list.push_back(node_info);
                        std::make_heap(open_list.begin(), open_list.end(), greater1<T1>());

//                        //该部分可以删除
//                        openlist_out_string << "openlist: \n" << "[" << node_info.self_cur_cor.x << ", " << node_info.self_cur_cor.y << "], " <<
//                                            node_info.self_g_value << ", " << node_info.self_f_value << ", [" << node_info.self_parent_cor.x << ", " << node_info.self_parent_cor.y << "]; \n";
//                        openlist_output << openlist_out_string.str() ;
//                        openlist_out_string.clear();
//                        openlist_out_string.str("");
                    }
                }
                else{
                    double cur_g_value = open_item.self_g_value + distance(cur_node.x, cur_node.y, node.x, node.y);
                    double cur_h_value = heuristic(node.x, node.y, target_point.x, target_point.y);
                    Record_Item<T1> node_info(node, cur_g_value,  (cur_g_value + cur_h_value), cur_node, open_list.size());
                    open_list.push_back(node_info);
                    std::make_heap(open_list.begin(), open_list.end(), greater1<T1>());

//                    //该部分可以删除
//                    openlist_out_string << "openlist: \n" << "[" << node_info.self_cur_cor.x << ", " << node_info.self_cur_cor.y << "], " <<
//                                        node_info.self_g_value << ", " << node_info.self_f_value << ", [" << node_info.self_parent_cor.x << ", " << node_info.self_parent_cor.y << "]; \n";
//                    openlist_output << openlist_out_string.str() ;
//                    openlist_out_string.clear();
//                    openlist_out_string.str("");
                }
            }
        }
    }
    std::vector<Node<T1> > path;
    double cost = -1.0;

    std::string output_filename = R"(../output/record_log.txt)";
    std::fstream record_output(output_filename, std::ios_base::out | std::ios_base::app);
    std::stringstream out_string;
    out_string << "strat_point-target_point: [" << start_point.x << ", " << start_point.y << "]-[" << target_point.x << ", " << target_point.y << "]" << "\t ";
    out_string << "open_list size: " << open_list.size() << "\t close_list size: " << close_list.size() << "\t" ;
    out_string << "path_size: " << path.size() << std::endl ;
    record_output << out_string.str() << std::endl;
    record_output.close();

    open_list.clear();
    open_list.shrink_to_fit();
    close_list.clear();
    close_list.shrink_to_fit();
    for(int i = 0; i < map_xsize; i++)
        delete []map[i];
    delete []map;

    return std::make_pair(path, cost);
}

template <class T1, class T2>
std::vector<Node<T1> > Astar<T1, T2>::get_neighbors(T1** map, int map_xsize, int map_ysize, Node<T1> cur, int method){
    std::vector<Node<T1> > neighbors;
    T1 cur_x = cur.x;
    T1 cur_y = cur.y;
    T1 neighbor_x = (T1)0;
    T1 neighbor_y = (T1)0;
    if (method == 4 || method == 8){
        for (int i = 0; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map))
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
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map) && is_valid_move(map, cur, Node<int>(neighbor_x, neighbor_y), map_xsize, map_ysize))
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
        }
    }
    return neighbors;
};

template <class T1, class T2>
std::vector<Node<T1> > Astar<T1, T2>::get_neighbors_new(T1** map, int map_xsize, int map_ysize, Node<T1> cur, int method){
    std::vector<Node<T1> > neighbors;
    T1 cur_x = cur.x;
    T1 cur_y = cur.y;
    T1 neighbor_x = (T1)0;
    T1 neighbor_y = (T1)0;
    if (method == 4 || method == 8){
        for (int i = 0; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map))
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
            T1 tmp_x = (T1)0;
            T1 tmp_y = (T1)0;
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map)){
                int size = new_offsets[i][8];
                int flag = 1;
                for (int j = 0; j < size; j++){
                    tmp_x = cur_x + new_offsets[i][2 * j];
                    tmp_y = cur_y + new_offsets[i][2 * j + 1];
                    if (!is_valid_neighbor(tmp_x, tmp_y, map_xsize, map_ysize, map)){
                        flag = 0;
                        break;
                    }
                }
                if (flag){
                    neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
                }
            }
        }
    }
    return neighbors;
};


/**
 * @brief
 * @tparam T
 * @param x x_cor
 * @param y y_cor
 * @param map_xsize
 * @param map_ysize
 * @return is valid neighbor return true; else return false;
 */
template <class T1, class T2>
bool Astar<T1, T2>::is_valid_neighbor(T1 x, T1 y, int map_xsize, int map_ysize, T1** map) {
    if( x >= 0 && x < map_xsize && y >= 0 && y < map_ysize && map[x][y] == 0) // map [y][x]
        return true;
    return false;
}

template <class T1, class T2>
double Astar<T1, T2>::heuristic(T1 x1_cor, T1 y1_cor, T1 x2_cor, T1 y2_cor){
    double h = sqrt(pow((x1_cor-x2_cor), 2) + pow((y1_cor-y2_cor), 2));
    return h;
};

template <class T1, class T2>
double Astar<T1, T2>::distance(T1 x1_cor, T1 y1_cor, T1 x2_cor, T1 y2_cor){
    double d = sqrt(pow((x1_cor-x2_cor), 2) + pow((y1_cor-y2_cor), 2));
    return d;
};

/**
 * @brief get point on line
 * @tparam T
 * @param sp
 * @param ep
 * @return
 */
template <class T1, class T2>
std::vector<Node<T1> > Astar<T1, T2>::get_lines(Node<T1> sp, Node<T1> ep){
    std::vector<Node<T1> > line;
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
            line.push_back(Node<int>(x1, i));
        }
    }
    else if(y1 == y2){
        if(x1 > x2){
            int tmp = x1;
            x1 = x2;
            x2 = tmp;
        }
        for(int i = x1; i < x2; i++){
            line.push_back(Node<int>(i, y1));
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
            if (!is_in_line(line, Node<int>(tmp_x, tmp_y)))
                line.push_back(Node<int>(tmp_x, tmp_y));
            xi = xi + 0.1;
        }
    }
    return line;
};


template <class T1, class T2>
bool Astar<T1, T2>::is_in_line(std::vector<Node<T1>> line, Node<T1> p) {
    for (Node<T1> i: line){
        if (i.x == p.x && i.y == p.y)
            return true;
    }
    return false;
}


template <class T1, class T2>
bool Astar<T1, T2>::is_valid_move(T1 **map, Node<T1> sp, Node<T1> ep, int map_xsize, int map_ysize) {
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
            if (!is_valid_neighbor(tmp_x, tmp_y, map_xsize, map_ysize, map))    //obstacle
                return false;
            xi = xi + 0.1;
        }
    }
    return true;
}

template <class T1, class T2>
bool Astar<T1, T2>::is_valid_move_new(T1 **map, Node<T1> cur, int map_xsize, int map_ysize, int i){
    int size = new_offsets[i][8];
    T1 cur_x = cur.x;
    T1 cur_y = cur.y;
    T1 neighbor_x = (T1)0;
    T1 neighbor_y = (T1)0;
    for (int j = 0; j < size; j++){
        neighbor_x = cur_x + new_offsets[i][2 * j];
        neighbor_y = cur_y + new_offsets[i][2 * j + 1];
        if (!is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map)){
            return false;
        }
    }
    return true;
}

/*template <class T1, class T2>
bool Astar<T1, T2>::is_valid_move_new(T1 **map, Node<T1> sp, Node<T1> ep) {
    int x1 = (int) sp.x;
    int x2 = (int) ep.x;
    int y1 = (int) sp.y;
    int y2 = (int) ep.y;
    double dx = x1 - x2;
    double dy = y1 - y2;
    double f = 0;
    int sx = 1;
    int sy = 1;
    if (dy < 0.0){
        dy = -dy;
        sy = -1;
    }
    if (dx < 0.0){
        dx = -dx;
        sx = -1;
    }
    if (dx >= dy){
        while(x1 != x2){
            f = f + dy;
            if (f >= dx){
                if (map[x1 + ((sx-1) / 2)][y1 + ((sy-1) / 2)])
                    return false;
                y1 = y1 + sy;
                f = f - dx;
            }
            if ((f != 0.0) && map[x1+((sx-1) / 2)][y1+((sy-1) / 2)])
                return false;
            if ((dy == 0.0) && map[x1+((sx-1) / 2)][y1] && map[x1+((sx-1) / 2)][y1-1])
                return false;
            x1 = x1 + sx;
        }
    } else{
        while (y1 != y2){
            f = f + dx;
            if (f >= dy){
                if (map[x1 + ((sx-1) / 2)][y1 + ((sy-1) / 2)])
                    return false;
                x1 = x1 + sx;
                f = f - dy;
            }
            if ((f != 0) && map[x1 + ((sx-1) / 2)][y1 + ((sy-1) / 2)])
                return false;
            if ((dx == 0.0) && map[x1][y1 + (sy-1) / 2] && map[x1 - 1][y1 + ((sy-1) / 2)])
                return false;
            y1 = y1 + sy;
        }
    }
    return true;
}*/

template <class T1, class T2>
int Astar<T1, T2>::is_in_list(std::vector<Record_Item<T1>> list, Node<T1> cur_node) {
    for(int i = 0; i < list.size(); i++){
        Node<T1> tmp_node = list[i].self_cur_cor;
        if (cur_node.x == tmp_node.x && cur_node.y == tmp_node.y)
            return i;
    }
    return -1;
}

template <class T1, class T2>
bool Astar<T1, T2>::is_in_open_list(std::vector<Open_list_Item<T1>> open_list, Node<T1> cur_node) {
    for (Open_list_Item<T1> item: open_list){
        Node<T1> tmp_node = item.self_cur_cor;
    }
    return false;
}

template <class T1, class T2>
std::pair<std::vector<Node<T1> >, double> Astar<T1, T2>::construct_path(std::vector<Record_Item<T1>> close_list,
                                                                        Node<T1> cur_node) {
    std::vector<Node<T1> > path;
    double cost = 0.0;
    Record_Item<T1> item = close_list.back();
    if (!((item).self_cur_cor.x == cur_node.x && (item).self_cur_cor.x == cur_node.x)) {

    }
    cost = (item).self_g_value;
    int parent_index = 0;
    while(!((item).self_cur_cor.x == (item).self_parent_cor.x && (item).self_cur_cor.y == (item).self_parent_cor.y)){
        parent_index = (item).self_parent_index;
        Node<T1> node((item).self_cur_cor);
        path.push_back(node);
        item = close_list.at(parent_index);
    }
    std::pair<std::vector<Node<T1> >, double> result = std::make_pair(path, cost);
    path.clear();
    path.shrink_to_fit();
    return result;
}

#endif //PATHPLANNING_ASTAR_HPP
