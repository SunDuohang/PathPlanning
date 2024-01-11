/**
  ******************************************************************************
  * @file           : AStar.cpp
  * @author         : SUNX
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/19
  * @last_change    : 
  ******************************************************************************
  */

#include "time.h"
#include <malloc.h>
#include "../include/AStar.h"

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

template <class T1, class T2>
std::pair<std::vector<Node<T1> >, double> Astar<T1, T2>::run(const char* filename,Node<T1>start_point, Node<T1> target_point, int method){
    T1** map;
    int map_xsize = 0;
    int map_ysize = 0;
    get_map_size(filename, map_xsize, map_ysize);
    map  = new T1 * [map_xsize];
    for(int i = 0; i < map_xsize; i++)
        map[i] = new T1 [map_ysize];
    load_map(filename, map, map_xsize, map_ysize);
    std::vector<Record_Item<T1> > close_list;
    std::vector<Record_Item<T1> > open_list;
    Node<T1> self_start_point = start_point;
    Node<T1> self_target_point = target_point;

    double g_value = 0.0;
    double h_value = heuristic(start_point.x, start_point.y, target_point.x, target_point.y);
    double f_value = g_value + h_value;
    Record_Item<T1> start_info(start_point, g_value, f_value, start_point, 0);
    open_list.push_back(start_info);
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
        }
        else{
            Record_Item<T1> close_item(open_item, parentnode_index);
            close_list.push_back(close_item);
        }

        if (target_point.x == cur_node.x && target_point.y == cur_node.y){
            auto result = construct_path(close_list, cur_node);
            open_list.clear();
            open_list.shrink_to_fit();
            close_list.clear();
            close_list.shrink_to_fit();
            return result;
        }

        std::vector<Node<T1> > neighbors = get_neighbors(map, map_xsize, map_ysize, cur_node, method);
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
                    }
                }
                else{
                    double cur_g_value = open_item.self_g_value + distance(cur_node.x, cur_node.y, node.x, node.y);
                    double cur_h_value = heuristic(node.x, node.y, target_point.x, target_point.y);
                    Record_Item<T1> node_info(node, cur_g_value,  (cur_g_value + cur_h_value), cur_node, open_list.size());
                    open_list.push_back(node_info);
                    std::make_heap(open_list.begin(), open_list.end(), greater1<T1>());
                }
            }
        }
    }
    std::vector<Node<T1> > path;
    double cost = -1.0;
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
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map) && is_valid_move(cur, Node<int>(neighbor_x, neighbor_y), map))
                neighbors.push_back(Node<T1>(neighbor_x, neighbor_y));
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
    if( x >= 0 && x < map_xsize && y >= 0 && y < map_ysize && map[x][y] == 0)
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
    ;
    return false;
}


template <class T1, class T2>
bool Astar<T1, T2>::is_valid_move(Node<T1> sp, Node<T1> ep, T1 **map) {
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

//template <class T1, class T2>
//int Astar<T1, T2>::is_in_list(std::vector<T2> list, Node<T1> cur_node){
//    for (int i = 0; i < list.size(); i++){
//        Node<T1> tmp_node = list[i].self_cur_cor;
//        if (cur_node.x == tmp_node.x && cur_node.y == tmp_node.y)
//            return i;
//    }
//    return -1;
//};

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