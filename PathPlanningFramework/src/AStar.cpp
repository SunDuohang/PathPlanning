/**
  ******************************************************************************
  * @file           : AStar.cpp
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-6
  ******************************************************************************
  */

#include "../include/AStar.h"

const double AStar::costLow = 1.0;
const double AStar::costHigh = 1.4;

double AStar::calculateH(std::shared_ptr<ANode> point, std::shared_ptr<ANode> end) const
{
    return costLow * (std::abs(point->x - end->x) + std::abs(point->y - end->y));
}
double AStar::calculateF(std::shared_ptr<ANode> point,std::shared_ptr<ANode> end) const
{
    return point->g_value + calculateH(point, end);
}
void AStar::HeapSort(int beg, int end)
{
    auto temp = openlist[beg];
    int pre = beg;
    for (int i = pre * 2 + 1; i <= end; i = i * 2 + 1)
    {
        if (i + 1 <= end && openlist[i + 1]->f_value < openlist[i]->f_value)
            ++i;
        if (openlist[i]->f_value <= temp->f_value)
            openlist[pre] = openlist[i];
        else
            break;
        pre = i;
    }
    openlist[pre] = temp;
}
void AStar::HeapSort(int end)
{
    int pre = int(end) - 1;
    int i = ((int(end) - 1) % 2 == 0) ? (int(end) - 1) / 2 - 1 : (int(openlist.size()) - 1) / 2;
    for (; i >= 0; i = (i % 2 == 0) ? i / 2 - 1 : i / 2)
    {
        if (openlist[i]->f_value >= openlist[pre]->f_value)
            std::swap(openlist[i], openlist[pre]);
        else
            break;
        pre = i;
    }
}
std::shared_ptr<ANode> AStar::findPath(std::shared_ptr<ANode> beg, std::shared_ptr<ANode> end)
{
    refreshOpenList(beg,end);
    while (!openlist.empty())
    {
        auto iter2 = std::find_if(openlist.cbegin(), openlist.cend(), [end](std::shared_ptr<ANode> sp)
        {return (sp->x == end->x) && (sp->y == end->y); });
        if (iter2 != openlist.end())
            return *iter2;
        auto iter_temp = openlist.front();
        std::swap(openlist[0], openlist[openlist.size() - 1]);
        openlist.pop_back();
        HeapSort(0,openlist.size()-1);
        closeist.push_back(iter_temp);
        refreshOpenList(iter_temp, end);
    }
    openlist.clear();
    closeist.clear();
    openlist.shrink_to_fit();
    for(auto it = closeist.begin(); it != closeist.end(); ){
        closeist.erase(it++);
    }
    return nullptr;
}
void AStar::refreshOpenList(std::shared_ptr<ANode> point, std::shared_ptr<ANode> end)
{
    bool upIsWall = false;//表示当前点上有障碍物，即对应的斜向没法走
    bool downIsWall = false;
    bool leftIsWall = false;
    bool rightIsWall = false;
    if (point->x - 1 >= 0 && maps[point->x - 1][point->y] == 1)
        upIsWall = true;
    if (point->x + 1 < int(maps.size()) && maps[point->x + 1][point->y] == 1)
        downIsWall = true;
    if (point->y - 1 >= 0 && maps[point->x][point->y - 1] == 1)
        leftIsWall = true;
    if (point->y + 1 < int(maps.front().size()) && maps[point->x][point->y + 1] == 1)
        rightIsWall = true;
    //第一步初始化起点及其周围的节点
    if (openlist.empty() && closeist.empty())
    {
        closeist.push_back(point);
        for (int i = point->x - 1; i <= point->x + 1; ++i)
        {
            for (int j = point->y - 1; j <= point->y + 1; ++j)
            {
                if (i >= 0 && j >= 0 && i < int(maps.size()) && j < int(maps.front().size()) && (i != point->x || j != point->y) && !maps[i][j])
                {
                    if (i != point->x && j != point->y)
                    {
                        if (leftIsWall && j < point->y)
                            continue;
                        if (rightIsWall && j > point->y)
                            continue;
                        if (upIsWall && i < point->x)
                            continue;
                        if (downIsWall && i > point->x)
                            continue;
                    }
                    auto cur = std::make_shared<ANode>(i, j, point);
                    cur->g_value = (i != point->x && j != point->y) ? costHigh : costLow;
                    cur->h_value = calculateH(cur, end);
                    cur->f_value = calculateF(cur, end);
                    openlist.push_back(cur);
                }
            }
        }
        int i = ((int(openlist.size()) - 1) % 2 == 0) ? (int(openlist.size()) - 1) / 2 - 1 : (int(openlist.size()) - 1) / 2;
        for (; i >= 0; --i)
            HeapSort(i, openlist.size() - 1);
    }
        //刷新每一次找到的起点到当前点的最短路径中的当前点的周围节点情况
    else
    {

        for (int i = point->x - 1; i <= point->x + 1; ++i)
        {
            for (int j = point->y - 1; j <= point->y + 1; ++j)
            {
                if (i >= 0 && j >= 0 && i < int(maps.size()) && j < int(maps.front().size()) && (i != point->x || j != point->y) && !maps[i][j])
                {
                    if (i != point->x && j != point->y)
                    {
                        if (leftIsWall && j < point->y)
                            continue;
                        if (rightIsWall && j > point->y)
                            continue;
                        if (upIsWall && i < point->x)
                            continue;
                        if (downIsWall && i > point->x)
                            continue;
                    }
                    auto cur = std::make_shared<ANode>(i, j, point);
                    cur->g_value = ((i != point->x && j != point->y) ? costHigh : costLow) + point->g_value;
                    cur->h_value = calculateH(cur, end);
                    cur->f_value = calculateF(cur, end);
                    auto iter_close=std::find_if(closeist.cbegin(), closeist.cend(), [i,j](std::shared_ptr<ANode> sp)
                    { return (sp->x == i) && (sp->y == j); });
                    if (iter_close == closeist.end())
                    {
                        auto iter_open=std::find_if(openlist.cbegin(), openlist.cend(), [i,j](std::shared_ptr<ANode> sp)
                        { return (sp->x == i) && (sp->y == j); });
                        if (iter_open != openlist.end())
                        {
                            if((*iter_open)->g_value > cur->g_value)
                            {
                                (*iter_open)->g_value = cur->g_value;
                                (*iter_open)->f_value = (*iter_open)->g_value + (*iter_open)->h_value;
                                (*iter_open)->prev = point;
                                HeapSort(int(iter_open - openlist.begin()) + 1);
                            }
                        }
                        else
                        {
                            openlist.push_back(cur);
                            HeapSort(int(openlist.size()));
                        }
                    }
                }
            }
        }
    }
}
std::pair<std::vector<Node >, double> AStar::PrintAStarPath(const std::pair<int, int>& start, const std::pair<int, int>& end)
{
    auto start_sp = std::make_shared<ANode>(start.first, start.second), end_sp = std::make_shared<ANode>(end.first, end.second);
    std::shared_ptr<ANode> cur = findPath(start_sp, end_sp);
    double cost = -1.0;
    std::vector<Node > path;
    if (!cur) {
        std::cout << "没有找到起点到终点路径" << std::endl;
    }
    else
    {
        cost = cur->f_value;
        while (cur)
        {
            Node node(cur->x, cur->y);
            path.push_back(node);
//            maps[cur->x][cur->y] = '*';
            cur = cur->prev.lock();
        }
    }
    openlist.clear();
    closeist.clear();
    openlist.shrink_to_fit();
    for(auto it = closeist.begin(); it != closeist.end(); ){
        closeist.erase(it++);
    }
    return std::make_pair(path, cost);
}

