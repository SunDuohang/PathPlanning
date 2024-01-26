/**
  ******************************************************************************
  * @file           : AStar_cmp.hpp
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-1-16
  * @last_change    :
  ******************************************************************************
  */
//

#ifndef PATHPLANNING_ASTAR_CMP_HPP
#define PATHPLANNING_ASTAR_CMP_HPP

#include <vector>
#include <algorithm>
#include <iostream>
#include <string>
#include <memory>
#include <malloc.h>
#include <cmath>
#include "./DataStructure.hpp"
#include "./DataLoad.h"

template <class T1, class T2>
class AStar_new{
    T1** self_map;
    int map_width;
    int map_height;
    std::string s;

    // offsets [][] = {{x_offset, y_offset}}
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

    short change_offsets_obs[24][9] = {{-1, 0, -1, -1, -2, -1, -2, -2, 4},
                                       {-1, 0, -1, -1, -2, -1, 0, 0, 2},
                                       {-1, 0, -2, 1, 0, 0, 0, 0, 2},
                                       {0, -1, -1, -1, -1, -2, -2, -2, 4},
                                       {0, -1, -1, -1, 0, 0, 0, 0, 2},
                                       {0, -1, -1, -2, 0, 0, 0, 0, 2},
                                       {0, -1, 1, -1, 1, -2, 2, -2, 4},
                                       {0, -1, 1, -1, 0, 0, 0, 0, 2},
                                       {0, -1, 1, -2, 0, 0, 0, 0, 2},
                                       {1, 0, 1, -1, 2, -1, 2, -2, 4},
                                       {1, 0, 1, -1, 0, 0, 0, 0, 2},
                                       {1, 0, 2, -1, 0, 0, 0, 0, 2},
                                       {1, 0, 1, 1, 2, 1, 2, 2, 4},
                                       {1, 0, 1, 1, 0, 0, 0, 0, 2},
                                       {1, 0, 2, 1, 0, 0, 0, 0, 2},
                                       {0, 1, 1, 1, 1, 2, 2, 2, 4},
                                       {0, 1, 1, 1, 0, 0, 0, 0, 2},
                                       {0, 1, 1, 2, 0, 0, 0, 0, 2},
                                       {0, 1, -1, 1, -1, 2, -2, 2, 4},
                                       {0, 1, -1, 1, 0, 0, 0, 0, 2},
                                       {0, 1, -1, 2, 0, 0, 0, 0, 2},
                                       {-1, 0, -1, 1, -2, 1, -2, 2, 4},
                                       {-1, 0, -1, 1, 0, 0, 0, 0, 2},
                                       {-1, 0, -2, 1, 0, 0, 0, 0, 2}};


    short outside_offsets[24][2] = {{-3, -2}, {-2, -1}, {-3, -1},
                                    {-2, -3}, {-1, -2}, {-1, -3},
                                    {2, -3}, {1, -2}, {1, -3},
                                    {3, -2}, {2, -1}, {3, -1},
                                    {3, 2}, {2, 1}, {3, 1},
                                    {2, 3}, {1, 2}, {1, 3},
                                    {-2, 3}, {-1, 2}, {-1, 3},
                                    {-3, 2}, {-2, 1}, {-3, 1},};

    short change_offsets[17][2] = {{0, -1},{1, 0}, {0, 1}, {-1, 0},
                                   {1, -1}, {1, 1}, {-1, 1}, {-1, -1},
                                   {0, 0}, {0, 0}, {0, 0}, {0, 0},
                                   {0, 0}, {0, 0}, {0, 0}, {0, 0},
                                   {0, 0}};

    short change_labels[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

public:
    AStar_new();
    explicit AStar_new(const char* filename);
    ~AStar_new();
    //
    std::pair<std::vector<Node<T1> >, double> run(const char* filename,Node<T1> start_point, Node<T1> target_point, int method=16);
    int is_in_list(std::vector<Record_Item<T1> > list, Node<T1> cur_node);
    std::pair<std::vector<Node<T1> >, double> construct_path(std::vector<Record_Item<T1> > close_list, Node<T1> cur_node);

    std::vector<Node<T1> > get_neighbors(T1** map, int width, int height, Node<T1> cur,int method=16);
    std::vector<Node<T1> > get_neighbors_new(T1** map, int width, int height, Node<T1> cur,int method=16);
    std::vector<Node<T1> > get_neighbors_oritanted(T1** map, int width, int height, Node<T1> cur,const short flags[], int lens);

    // check vaild
    bool is_valid_move(T1 **map, Node<T1> sp, Node<T1> ep, int width, int height);
    bool is_valid_move_new(T1 **map, Node<T1> cur, int width, int height, int i);

    bool is_valid_neighbor(T1 **map, T1 x, T1 y, int width, int height);

    // 启发式
    double distance(T1 x1_cor, T1 y1_cor, T1 x2_cor, T1 y2_cor);
    double heuristic(T1 x1_cor, T1 y1_cor, T1 x2_cor, T1 y2_cor);

    int get_target_angle_seg(const Node<int>& sp, const Node<int>& ep);

    void set_changeable_offsets(const Node<int>& sp, const Node<int>& ep);
};



template <class T1, class T2>
AStar_new<T1, T2>::AStar_new() {
    map_width = 0;
    map_height = 0;
}

template <class T1, class T2>
AStar_new<T1, T2>::AStar_new(const char *filename) {
    get_map_size(filename, map_width, map_height);
    self_map = new T1* [map_height];
    for(int i = 0; i < map_height; i++)
        self_map[i] = new T1 [map_width];
    load_map(filename, self_map, map_width, map_height);
}

template <class T1, class T2>
AStar_new<T1, T2>::~AStar_new() {
    for(int i = 0; i < map_height; i++)
        delete []self_map[i];
    delete []self_map;
    malloc_trim(0);
}

template <class T1, class T2>
std::pair<std::vector<Node<T1>>, double> AStar_new<T1, T2>::run(const char *filename, Node<T1> start_point, Node<T1> target_point, int method) {
    T1** map;
    int mapWidth = 0;
    int mapHeight = 0;
    get_map_size(filename, mapWidth, mapHeight);
    map = new T1* [mapHeight];
    for(int i = 0; i < mapHeight; i++)
        map[i] = new T1 [mapWidth];
    load_map(filename, map, mapWidth, mapHeight);

    std::vector<Record_Item<T1> > close_list;
    std::vector<Record_Item<T1> > open_list;

//    set_changeable_offsets(start_point, target_point);

    // 该部分不可以删除
    double g_value = 0.0;
    double h_value = heuristic(start_point.x, start_point.y, target_point.x, target_point.y);
    double f_value = g_value + h_value;
    Record_Item<T1> start_info(start_point, g_value, f_value, start_point, 0);
    open_list.push_back(start_info);

    std::make_heap(open_list.begin(), open_list.end(), greater1<T1>());
    while(!open_list.empty()){
        Record_Item<T1> open_item = open_list[0];
        std::pop_heap(open_list.begin(), open_list.end(), greater1<T1>());
        open_list.pop_back();
        Node<T1> cur_node = open_item.self_cur_cor;
        Node<T1> parent_node = open_item.self_parent_cor;

        int parentnode_index = is_in_list(close_list, parent_node);
        if (parentnode_index == -1){
            Record_Item<T1> close_item(open_item, 0);
            close_list.push_back(close_item);
        }
        else{
            Record_Item<T1> close_item(open_item, parentnode_index);
            close_list.push_back(close_item);
        }

        // 如果查询到，返回结果
        if (target_point.x == cur_node.x && target_point.y == cur_node.y){
            auto result = construct_path(close_list, cur_node);
            for(int i = 0; i < mapHeight; i++)
                delete []map[i];
            delete []map;

            open_list.clear();
            open_list.shrink_to_fit();
            close_list.clear();
            close_list.shrink_to_fit();
            return result;
        }

        std::vector<Node<T1> > neighbors = get_neighbors_oritanted(map, mapWidth, mapHeight, cur_node, change_labels, 9);
        //std::vector<Node<T1> > neighbors = get_neighbors_new(map, mapWidth, mapHeight, cur_node, method);
        for(Node<T1> node: neighbors){
            if (is_in_list(close_list, node) != -1)
                continue;
            int open_list_index = is_in_list(open_list, node);
            if (open_list_index != -1){
                double cur_g_value = open_item.self_g_value + distance(cur_node.x, cur_node.y, node.x, node.y);
                if (cur_g_value  < open_list[open_list_index].self_g_value){
                    double cur_h_value = heuristic(node.x, node.y, target_point.x, target_point.y);
                    open_list.erase(open_list.begin()+open_list_index);
                    Record_Item<T1> node_info(node, cur_g_value, (cur_g_value+cur_h_value), cur_node, open_list.size()); // 这里似乎有一个问题
                    open_list.push_back(node_info);
                    std::make_heap(open_list.begin(), open_list.end(), greater1<T1>());
                }
            }
            else{
                double cur_g_value = open_item.self_g_value + distance(cur_node.x, cur_node.y, node.x, node.y);
                double cur_h_value = heuristic(node.x, node.y, target_point.x, target_point.y);
                Record_Item<T1> node_info(node, cur_g_value, (cur_g_value+cur_h_value), cur_node, open_list.size());
                open_list.push_back(node_info);
                std::make_heap(open_list.begin(), open_list.end(), greater1<T1>());
            }
        }
    }
    std::vector<Node<T1> > path;
    double cost = -1.0;

    open_list.clear();
    open_list.shrink_to_fit();
    close_list.clear();
    close_list.shrink_to_fit();
    // release memory
    for (int i = 0; i < mapHeight; i++)
        delete []map[i];
    delete []map;

    return std::make_pair(path, cost);
}

template<class T1, class T2>
int AStar_new<T1, T2>::is_in_list(std::vector<Record_Item<T1>> list, Node<T1> cur_node) {
    for(int i = 0; i < list.size(); i++){
        Node<T1> tmp_node = list[i].self_cur_cor;
        if (cur_node.x == tmp_node.x && cur_node.y == tmp_node.y)
            return i;
    }
    return -1;
}

template<class T1, class T2>
std::pair<std::vector<Node<T1>>, double>
AStar_new<T1, T2>::construct_path(std::vector<Record_Item<T1>> close_list, Node<T1> cur_node) {
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

template<class T1, class T2>
std::vector<Node<T1>> AStar_new<T1, T2>::get_neighbors(T1 **map, int width, int height, Node<T1> cur, int method) {
    std::vector<Node<T1> > neighbors;
    T1 cur_x = cur.x;
    T1 cur_y = cur.y;
    T1 neighbor_x = (T1)0;
    T1 neighbor_y = (T1)0;
    if(method == 4 || method == 8){
        for(int i = 0; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if(is_valid_neighbor(map, neighbor_x, neighbor_y, width, height))
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
        }
    }
    else if(method == 16 || method == 32){
        for (int i = 0; i < 8; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(map, neighbor_x, neighbor_y, width, height))
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
        }
        for (int i = 8; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if(is_valid_neighbor(map, neighbor_x, neighbor_y, width, height) && is_valid_move(map, cur, Node<int>(neighbor_x, neighbor_y), width, height))
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
        }
    }
    return neighbors;
}


/**
 * @brief 获取周围邻居
 * @tparam T1
 * @tparam T2
 * @param map
 * @param width
 * @param height
 * @param cur
 * @param method
 * @return
 */
template<class T1, class T2>
std::vector<Node<T1>>
AStar_new<T1, T2>::get_neighbors_new(T1 **map, int width, int height, Node<T1> cur, int method) {
    std::vector<Node<T1> > neighbors;
    T1 cur_x = cur.x;
    T1 cur_y = cur.y;
    T1 neighbor_x = (T1)0;
    T1 neighbor_y = (T1)0;
    if (method == 4 || method == 8){
        for (int i = 0; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(map, neighbor_x, neighbor_y, width, height))
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
        }
    }
    else if(method == 16 || method == 32){
        for (int i = 0; i < 8; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(map, neighbor_x, neighbor_y, width, height))
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
        }
        for (int i = 8; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            T1 tmp_x = (T1)0;
            T1 tmp_y = (T1)0;
            if (is_valid_neighbor(map, neighbor_x, neighbor_y, width, height)){
                int size = new_offsets[i][8];
                int flag = 1;
                for (int j = 0; j < size; j++){
                    tmp_x = cur_x + new_offsets[i][2 * j];
                    tmp_y = cur_y + new_offsets[i][2 * j + 1];
                    if (!is_valid_neighbor(map, tmp_x, tmp_y, width, height)){
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
}

template<class T1, class T2>
std::vector<Node<T1>>
AStar_new<T1, T2>::get_neighbors_oritanted(T1 **map, int width, int height, Node<T1> cur, const short *flags, int lens) {
    std::vector<Node<T1> > neighbors;
    T1 cur_x = cur.x;
    T1 cur_y = cur.y;
    T1 neighbor_x = (T1)0;
    T1 neighbor_y = (T1)0;
    for (int i = 0; i < 8; i++){
        neighbor_x = cur_x + offsets[i][0];
        neighbor_y = cur_y + offsets[i][1];
        if (is_valid_neighbor(map, neighbor_x, neighbor_y, width, height))
            neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
    }

    for (int i = 0; i < lens; i++){
        short index = flags[i];
        neighbor_x = cur_x + change_offsets[index][0];
        neighbor_y = cur_y + change_offsets[index][1];
        T1 tmp_x = (T1)0;
        T1 tmp_y = (T1)0;
        if (is_valid_neighbor(map, neighbor_x, neighbor_y, width, height)){
            int size = change_offsets_obs[index][8];
            int flag = 1;
            for (int j = 0; j < size; j++){
                tmp_x = cur_x + change_offsets_obs[index][2 * j];
                tmp_y = cur_y + change_offsets_obs[index][2 * j + 1];
                if (!is_valid_neighbor(map, tmp_x, tmp_y, width, height)){
                    flag = 0;
                    break;
                }
            }
            if (flag){
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
            }
        }
    }
    return neighbors;
}

template<class T1, class T2>
bool AStar_new<T1, T2>::is_valid_move(T1 **map, Node<T1> sp, Node<T1> ep, int width, int height) {
    double slope = 0.0;
    int x1 = (int)sp.x;
    int y1 = (int)sp.y;
    int x2 = (int)ep.x;
    int y2 = (int)ep.y;

    if(x1 == x2){
        if(y1 > y2){
            int tmp = y1;
            y1 = y2;
            y2 = tmp;
        }
        for(int i = y1; i < y2; i++){
            if(map[i][x1]) // map = 1 obstacles
                return false;
        }
    }
    else if (y1 == y2){
        if (x1 > x2){
            int tmp = x1;
            x1 = x2;
            x2 = tmp;
        }
        for (int i = x1; i < x2; i++){
            if (map[y1][i])
                return false;
        }
    }
    else if (sp.x > ep.x){
        x1 = ep.x;
        x2 = sp.x;
        y1 = ep.y;
        y2 = sp.y;
    }
    slope = 1.0 * (y2 - y1) / (x2 - x1);

    if(fabs((slope - 0.0)) > 1e-3){
        for (double xi = x1; xi < x2; ){
            int tmp_x = round(xi);
            int tmp_y = round(slope * (xi -x1) + y1);
            if(!is_valid_neighbor(map, tmp_x, tmp_y, width, height))    // obstacles
                return false;
            xi = xi + 0.1;
        }
    }
    return true;
}

template<class T1, class T2>
bool AStar_new<T1, T2>::is_valid_move_new(T1 **map, Node<T1> cur, int width, int height, int i) {
    int size = new_offsets[i][8];
    T1 cur_x = cur.x;
    T1 cur_y = cur.y;
    T1 neighbor_x = (T1) 0;
    T1 neighbor_y = (T1) 0;
    for (int j = 0; j < size; j++){
        neighbor_x = cur_x + new_offsets[i][2 * j];
        neighbor_y = cur_x + new_offsets[i][2 * j + 1];
        if (!is_valid_neighbor(map, neighbor_x, neighbor_y, width, height))
            return false;
    }
    return true;
}

/**
 *
 * @tparam T1
 * @tparam T2
 * @param map
 * @param x
 * @param y
 * @param width map_width int
 * @param height map_height int
 * @return
 */
template<class T1, class T2>
bool AStar_new<T1, T2>::is_valid_neighbor(T1 **map, T1 x, T1 y, int width, int height) {
    if (x >= 0 && x < width && y >= 0 && y < height && map[y][x] == 0)
        return true;
    return false;
}

template<class T1, class T2>
double AStar_new<T1, T2>::heuristic(T1 x1_cor, T1 y1_cor, T1 x2_cor, T1 y2_cor) {
    double h = sqrt(pow((x1_cor-x2_cor), 2) + pow((y1_cor-y2_cor), 2));
    return h;
}

template<class T1, class T2>
double AStar_new<T1, T2>::distance(T1 x1_cor, T1 y1_cor, T1 x2_cor, T1 y2_cor) {
    double d = sqrt(pow((x1_cor-x2_cor), 2) + pow((y1_cor-y2_cor), 2));
    return d;
}

template<class T1, class T2>
int AStar_new<T1, T2>::get_target_angle_seg(const Node<int> &sp, const Node<int> &ep) {
    double angle_rad = atan2((double)(ep.y -sp.y), (double)(ep.x - sp.x));
    double pi = M_PI;
    double angle_deg = (angle_rad / pi) * 180 + 180;
    int seg = (int)(angle_deg / 45);
    return (seg%8);
}

template<class T1, class T2>
void AStar_new<T1, T2>::set_changeable_offsets(const Node<int> &sp, const Node<int> &ep) {

    int seg = get_target_angle_seg(sp, ep);

    std::vector<int> tmp;
    int seg_start = (seg + 8 - 1) % 8;
    int seg_end = 3 * (seg_start + 1);
    change_labels[0] = seg_start * 3;
    change_labels[1] = seg_start * 3;
    change_labels[2] = seg_start * 3;
    for(int j = 3 * seg_start; j < seg_end; j++){
        tmp.push_back(outside_offsets[j][0]);
        tmp.push_back(outside_offsets[j][1]);
    }

    seg_start = seg;
    seg_end = 3 * (seg_start + 1);
    change_labels[3] = seg_start * 3;
    change_labels[4] = seg_start * 3;
    change_labels[5] = seg_start * 3;
    for(int j = 3 * seg_start; j < seg_end; j++){
        tmp.push_back(outside_offsets[j][0]);
        tmp.push_back(outside_offsets[j][1]);
    }

    seg_start = (seg + 8 + 1) % 8;
    seg_end = 3 * (seg_start + 1);
    change_labels[6] = seg_start * 3;
    change_labels[7] = seg_start * 3;
    change_labels[8] = seg_start * 3;
    for(int j = 3 * seg_start; j < seg_end; j++){
        tmp.push_back(outside_offsets[j][0]);
        tmp.push_back(outside_offsets[j][1]);
    }

    int size = tmp.size() / 2;
    for (int j = 0; j < size; j++){
        change_offsets[8+j][0] = tmp[2*j];
        change_offsets[8+j][1] = tmp[2*j+1];
    }

    tmp.clear();
    tmp.shrink_to_fit();
}

#endif //PATHPLANNING_ASTAR_CMP_HPP
