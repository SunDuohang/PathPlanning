//
// Created by sunx on 24-1-11.
//

#ifndef PATHPLANNING_DATASTRUCTURE_HPP
#define PATHPLANNING_DATASTRUCTURE_HPP

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
        self_cur_cor = Node<T>(cur_cor[0],cur_cor[1]);  //这里面有个问题就是，谁是x，谁是y
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


#endif //PATHPLANNING_DATASTRUCTURE_HPP
