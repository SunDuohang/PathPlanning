/**
  ******************************************************************************
  * @file           : AStar.h
  * @author         : SUNX
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/19
  * @last_change    : 
  ******************************************************************************
  */


#pragma once
#ifndef PATHPLANNING_ASTAR_H
#define PATHPLANNING_ASTAR_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <cmath>
#include <cstring>
#include <memory>
#include "./loadmap.h"
#include "./List.hpp"

template <class T>
struct Node{
    Node(T x_cor, T y_cor){
        x = x_cor;
        y = y_cor;
    }
    Node(){
        x = (T)0;
        y = (T)0;
    }

    Node(Node<T> const &node){
        x = node.x;
        y = node.y;
    }
    ~Node()=default;
    T x;
    T y;
};

template <class T>
class Record_Item{
public:
    Record_Item(Node<T> cur_cor, double g_value,  double f_value,Node<T> parent_cor, int p_index){
        self_cur_cor = cur_cor;
        self_g_value = g_value;
        self_f_value = f_value;
        self_parent_cor = parent_cor;
        self_parent_index = p_index;
    }
    Record_Item(const T cur_cor[], double g_value, double f_value,const T parent_cor[], int p_index){
//        if(2 != (sizeof(cur_cor) / sizeof(cur_cor[0]))){
//            printf("Error: cur_cor or parent_cor is wrong!\n");
//            exit(-1);
//        }
//        if(2 != (sizeof(parent_cor) / sizeof(parent_cor[0]))){
//            printf("Error: cur_cor or parent_cor is wrong!\n");
//            exit(-1);
//        }
        self_cur_cor = Node<T>(cur_cor[0],cur_cor[1]);
        self_g_value = g_value;
        self_f_value = f_value;
        self_parent_cor = Node<T>(parent_cor[0], parent_cor[1]);
        self_parent_index = p_index;
    }
    Record_Item(Record_Item<T> const &item, int p_index){
        self_cur_cor = Node<T>(item.self_cur_cor);
        self_g_value = item.self_g_value;
        self_f_value = item.self_f_value;
        self_parent_cor = Node<T>(item.self_parent_cor);
        self_parent_index = p_index;
    }

    ~Record_Item(){
        self_cur_cor.~Node();
        self_parent_cor.~Node();
    };
    Node<T> self_cur_cor;
    double self_g_value = 0.0;
    double self_f_value = 0.0;
    Node<T> self_parent_cor;
    int self_parent_index;
};

template <class T>
class Open_list_Item{
    Open_list_Item(const T cur_cor[], double g_value, double f_value){
        self_cur_cor = Node<T>(cur_cor[0],cur_cor[1]);
        self_g_value = g_value;
        self_f_value = f_value;
    }
    Open_list_Item(Node<T> cur_cor, double g_value, double f_value){
        self_cur_cor = cur_cor;
        self_g_value = g_value;
        self_f_value = f_value;
    }
    ~Open_list_Item()=default;
    Node<T> self_cur_cor;
    double self_g_value = 0.0;
    double self_f_value = 0.0;
};

template<class T>
struct greater1{
    constexpr bool operator()(Record_Item<T> lhs, const Record_Item<T>& rhs ) const{
        return lhs.self_f_value > rhs.self_f_value;
    }
};

template<class T>
struct greater2{
    constexpr bool operator()(Open_list_Item<T> lhs, const Open_list_Item<T>& rhs ) const{
        return lhs.self_f_value > rhs.self_f_value;
    }
};

template <class T>
struct less1{
    constexpr bool operator()(Record_Item<T> lhs, const Record_Item<T>& rhs ) const{
        return lhs.self_f_value < rhs.self_f_value;
    }
};

template <class T>
struct less2{
    constexpr bool operator()(Open_list_Item<T> lhs, const Open_list_Item<T>& rhs ) const{
        return lhs.self_f_value < rhs.self_f_value;
    }
};

template <class T>
struct equal1{
    constexpr bool operator()(Node<T> lhs, const Node<T>& rhs ) const{
        return (lhs.x == rhs.x && lhs.y == rhs.y);
    }
};


template <class T1, class T2>
class Astar{
private:
//    T map {std::make_unique<T **>()};
    T1** self_map;
    int map_Xsize{};
    int map_Ysize{};
    std::string s;
//    std::vector<Record_Item<T>> queue_;
    //std::map<std::string, std::vector<List>  > offsets_dict;
    int offsets[32][2] = {{0, -1,},{1, 0}, {0, 1}, {-1, 0},
                          {1, -1}, {1, 1}, {-1, 1}, {-1, -1},
                          {1, -2}, {2, -1},{2, 1},{1, 2},
                          {-1, 2}, {-2, 1}, {-2, -1}, {-1, -2},
                          {1, -3}, {2, -3}, {3, -2}, {3, -1},
                          {3, 1}, {3, 2}, {2, 3}, {1, 3},
                          {-1, 3}, {-2, 3}, {-3, 2}, {-3, 1},
                          {-3, -1}, {-3, -2}, {-2, -3}, {-1, -3}};


public:
    explicit Astar(const char* filename);
    ~Astar();
    //bool run(Node<T1> start_point, Node<T1> target_point, int method=16);
    std::pair<std::vector<Node<T1> >, double> run(const char* filename,Node<T1> start_point, Node<T1> target_point, int method=16);
    std::vector<Node<T1> > get_neighbors(T1** map, int map_xsize, int map_ysize, Node<T1> cur,int method=16);
    bool is_valid_neighbor(T1 x, T1 y, int map_xsize, int map_ysize, T1** map);
    double heuristic(T1 x1_cor, T1 y1_cor, T1 x2_cor, T1 y2_cor);
    double distance(T1 x1_cor, T1 y1_cor, T1 x2_cor, T1 y2_cor);
    std::vector<Node<T1> > get_lines(Node<T1> sp, Node<T1> ep);
    bool is_in_line(std::vector<Node<T1> > line, Node<T1>p);
    bool is_valid_move(Node<T1> sp, Node<T1> ep, T1** map);
    std::pair<std::vector<Node<T1> >, double> construct_path(std::vector<Record_Item<T1> > close_list, Node<T1> cur_node);
    int is_in_list(std::vector<Record_Item<T1> > close_list, Node<T1> cur_node);
    bool is_in_open_list(std::vector<Open_list_Item<T1> > open_list, Node<T1> cur_node);

};



#endif //PATHPLANNING_ASTAR_H
