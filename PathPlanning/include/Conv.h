//
// Created by sunx on 23-12-26.
//

#ifndef PATHPLANNING_CONV_H
#define PATHPLANNING_CONV_H

#include <iostream>
#include <map>
#include <cmath>

class Conv{
private:
    enum CONV_TYPE {
        ID_full = 0,
        ID_valid,
        ID_same,
    };
    std::map<std::string, int> convtype_str_id = std::map<std::string, int>();
    int matrix_w, matrix_h, kernel_size;

public:

    Conv(){
        convtype_str_id.insert(std::pair<std::string, int>("full", ID_full));
        convtype_str_id.insert(std::pair<std::string, int>("same", ID_same));
        convtype_str_id.insert(std::pair<std::string, int>("valid", ID_valid));
    }

    template<typename Tp>
    Tp* mul(Tp* matrix_a, Tp* matrix_b, int a_w, int a_h, int b_w, int b_h){};

    template<typename Tp>
    Tp* rot90(Tp* matrix, int length, int nums, bool show){};

    template<typename Tp>
    void conv(Tp* matrix, Tp* kernel, int matrix_w, int matrix_h, int kenel_length, Tp* result, bool show, bool is_rot, std::string conv_type="same"){};

    template<typename Tp>
    Tp* pad_matrix(Tp* matrix, int matrix_w, int matrix_h, int matrix_w_new, int matrix_h_new, bool show){};

    template<typename Tp>
    void show_matrix(Tp* matrix, int matrix_w, int matrix_h){};

};



#endif //PATHPLANNING_CONV_H
