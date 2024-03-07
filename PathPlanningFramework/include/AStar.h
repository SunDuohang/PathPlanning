/**
  ******************************************************************************
  * @file           : AStar.h
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-6
  ******************************************************************************
  */

#pragma once
#ifndef PATHPLANNINGFRAMEWORK_ASTAR_H
#define PATHPLANNINGFRAMEWORK_ASTAR_H

#include <utility>
#include <vector>
#include <list>
#include <memory>
#include <algorithm>
#include <iostream>
#include "./DataStructure.h"

class AStar
{
public:
    AStar(std::vector<std::vector<int>> m ): maps(std::move(m)) {}
    std::shared_ptr<ANode> findPath(std::shared_ptr<ANode> beg, std::shared_ptr<ANode> end);
    std::pair<std::vector<Node >, double> PrintAStarPath(const std::pair<int, int> &, const std::pair<int, int> &);
    ~AStar()
    {
        openlist.clear();
        closeist.clear();
        openlist.shrink_to_fit();
    }

private:
    void refreshOpenList(std::shared_ptr<ANode>, std::shared_ptr<ANode> end);
    double calculateH(std::shared_ptr<ANode>, std::shared_ptr<ANode>) const;
    double calculateF(std::shared_ptr<ANode>,std::shared_ptr<ANode>) const;
    void HeapSort(int beg, int end);
    void HeapSort(int end);

private:
    std::vector<std::vector<int>> maps;//地图
    std::vector<std::shared_ptr<ANode>> openlist;//保存还未遍历过的节点
    std::list<std::shared_ptr<ANode>> closeist;//保存已经找到最短路径的节点
    const static double costLow;//上下位移的距离
    const static double costHigh;//斜向位移的距离
};


#endif //PATHPLANNINGFRAMEWORK_ASTAR_H
