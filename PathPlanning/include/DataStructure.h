/**
  ******************************************************************************
  * @file           : DataStructure.h
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-1-25
  * @last_change    :
  ******************************************************************************
  */
//

#ifndef PATHPLANNING_DATASTRUCTURE_H
#define PATHPLANNING_DATASTRUCTURE_H

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <chrono>
#include <cmath>
#include <algorithm>

struct Node{
    Node(int x_cor, int y_cor){
        x = x_cor;
        y = y_cor;
    }
    Node(){
        x = 0;
        y = 0;
    }

    Node(Node const &node){
        x = node.x;
        y = node.y;
    }

    void operator=(const Node &node){
        x = node.x;
        y = node.y;
    }

    ~Node()=default;

    int x;
    int y;
};

struct LNode {
    int x, y;
    float g, h, f;
    LNode *parent;

    LNode(int x, int y) : x(x), y(y), g(0), h(0), f(0), parent(nullptr) {}

    // Calculate the heuristic value (Euclidean distance)
    void calculateHeuristic(LNode *end) {
        this->h = std::sqrt((end->x - this->x) * (end->x - this->x) + (end->y - this->y) * (end->y - this->y));
    }

    // Update the f value
    void updateCost() {
        this->f = this->g + this->h;
    }

    // Equality check based on coordinates
    bool operator==(const LNode &other) const {
        return x == other.x && y == other.y;
    }
};

// Custom hash function for Node
struct LNodeHash {
    size_t operator()(const LNode &node) const {
        return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
    }
};


class Record_Item{
public:
    Record_Item(const Node& cur_cor, double g_value,  double f_value,Node parent_cor, int p_index){
        self_cur_cor = cur_cor;
        self_g_value = g_value;
        self_f_value = f_value;
        self_parent_cor = parent_cor;
        self_parent_index = p_index;
    }
    Record_Item(const int cur_cor[], double g_value, double f_value,const int parent_cor[], int p_index){

        self_cur_cor = Node(cur_cor[0],cur_cor[1]);  //这里面有个问题就是，谁是x，谁是y
        self_g_value = g_value;
        self_f_value = f_value;
        self_parent_cor = Node(parent_cor[0], parent_cor[1]);
        self_parent_index = p_index;
    }
    Record_Item(Record_Item const &item, int p_index){
        self_cur_cor = Node(item.self_cur_cor);
        self_g_value = item.self_g_value;
        self_f_value = item.self_f_value;
        self_parent_cor = Node(item.self_parent_cor);
        self_parent_index = p_index;
    }

    ~Record_Item(){
        self_cur_cor.~Node();
        self_parent_cor.~Node();
    };
    Node self_cur_cor;
    double self_g_value = 0.0;
    double self_f_value = 0.0;
    Node self_parent_cor;
    int self_parent_index;
};


class Open_list_Item{
public:
    Open_list_Item(const int cur_cor[], double g_value, double f_value){
        self_cur_cor = Node(cur_cor[0],cur_cor[1]);
        self_g_value = g_value;
        self_f_value = f_value;
    }
    Open_list_Item(const Node& cur_cor, double g_value, double f_value){
        self_cur_cor = cur_cor;
        self_g_value = g_value;
        self_f_value = f_value;
    }
    ~Open_list_Item()=default;
    Node self_cur_cor;
    double self_g_value = 0.0;
    double self_f_value = 0.0;
};


struct greater1{
    constexpr bool operator()(const Record_Item& lhs, const Record_Item& rhs ) const{
        return lhs.self_f_value > rhs.self_f_value;
    }
};


struct greater2{
    constexpr bool operator()(const Open_list_Item& lhs, const Open_list_Item & rhs ) const{
        return lhs.self_f_value > rhs.self_f_value;
    }
};

template <class T>
struct less1{
    constexpr bool operator()(const Record_Item& lhs, const Record_Item& rhs ) const{
        return lhs.self_f_value < rhs.self_f_value;
    }
};

template <class T>
struct less2{
    constexpr bool operator()(const Open_list_Item& lhs, const Open_list_Item& rhs ) const{
        return lhs.self_f_value < rhs.self_f_value;
    }
};

template <class T>
struct equal1{
    constexpr bool operator()(const Node& lhs, const Node& rhs ) const{
        return (lhs.x == rhs.x && lhs.y == rhs.y);
    }
};

#endif //PATHPLANNING_DATASTRUCTURE_H
