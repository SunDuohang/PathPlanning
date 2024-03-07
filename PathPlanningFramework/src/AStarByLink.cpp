/**
  ******************************************************************************
  * @file           : AStarByLink.cpp
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-7
  ******************************************************************************
  */

#include <utility>

#include "../include/AStarByLink.h"

AStarByLink::AStarByLink(const char *filename) {
    gridMap.set_GridMap(filename);
    map_height = gridMap.self_height;
    map_width = gridMap.self_width;
}

std::pair<std::vector<Node>, double>
AStarByLink::run(const char *filename, Node &start_point, Node &target_point, int method) {
    GridMap gridMapi(filename);
    int width = gridMap.self_width;
    int height = gridMap.self_height;

    std::vector<List_Item> open_list;
    std::vector<List_Item> close_list;

    double g_value = 0.0;
    double h_value = Octi_heuristic(start_point.x, start_point.y, target_point.x, target_point.y);
    double f_value = g_value + h_value;
    auto start_info = std::make_shared<ANode> (start_point.x, start_point.y, g_value, h_value, f_value);
    start_info->prev = start_info;
    List_Item start_item(start_info, f_value);
    open_list.push_back(start_item);

    std::make_heap(open_list.begin(), open_list.end(), greater_link());
    while(!open_list.empty()){
        List_Item open_item = open_list[0];
        std::pop_heap(open_list.begin(), open_list.end(), greater_link());
        open_list.pop_back();
        std::shared_ptr<ANode> cur_node = open_item.cur;

        if (cur_node->x == target_point.x && target_point.y == cur_node->y){
            auto result = construct_path(cur_node);
            open_list.clear();
            open_list.shrink_to_fit();
            close_list.clear();
            close_list.shrink_to_fit();
            return result;
        }

        std::shared_ptr<ANode> parent = cur_node->prev.lock();
        int dx = cur_node->x - parent->x;
        int dy = cur_node->y - parent->y;
        std::vector<Node > neighbors = get_neighbors(gridMapi.self_map, width, height, cur_node, dx, dy, method);
        for(Node node: neighbors){

        }
    }

    std::vector<Node > path;
    double cost = -1.0;

    open_list.clear();
    open_list.shrink_to_fit();
    close_list.clear();
    close_list.shrink_to_fit();
    return std::make_pair(path, cost);
}

double AStarByLink::Euli_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor) {
    double h = sqrt(pow((x1_cor - x2_cor), 2) + pow((y1_cor - y2_cor), 2));
    return h;
}

double AStarByLink::Manh_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor) {
    return std::fabs(x2_cor - x1_cor) + std::fabs(y2_cor - y1_cor);
}

double AStarByLink::Cheb_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor) {
    return std::fmax(std::fabs(x2_cor - x1_cor), std::fabs(y2_cor - y1_cor));
}

double AStarByLink::Octi_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor) {
    double h = sqrt(2)*std::min(std::abs(x2_cor - x1_cor), std::abs(y2_cor - y1_cor));
    double s = std::abs(std::abs(x2_cor - x1_cor) - std::abs(y2_cor - y1_cor));
    return h+s;
}

double AStarByLink::distance(int x1_cor, int y1_cor, int x2_cor, int y2_cor) {
    double d = sqrt(pow((x1_cor - x2_cor), 2) + pow((y1_cor - y2_cor), 2));
    return d;
}

std::vector<Node> AStarByLink::get_neighbors(std::vector<std::vector<int> > &map, int width, int height, std::shared_ptr<ANode> cur, int method) {
    std::vector<Node > neighbors;
    int cur_x = cur->x;
    int cur_y = cur->y;
    int neighbor_x = 0;
    int neighbor_y = 0;
    if (method == 4 || method == 8){
        for (int i = 0; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
    }
    else if(method == 16 || method == 32){
        for (int i = 0; i < 8; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
        for (int i = 8; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y) && is_valid_move(map, width, height, Node(cur->x, cur->y), Node(neighbor_x, neighbor_y)))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
    }
    return neighbors;
}

std::vector<Node>
AStarByLink::get_neighbors(std::vector<std::vector<int> > &map, int width, int height, std::shared_ptr<ANode> cur, int dx, int dy, int method) {
    std::vector<Node > neighbors;
    int cur_x = cur->x;
    int cur_y = cur->y;
    int neighbor_x = 0;
    int neighbor_y = 0;
    if (method == 4){
        for (int i = 0; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
    }
    if (method == 8){
        neighbor_x = cur_x + dx;
        neighbor_y = cur_y + dy;
        int tmp_x = neighbor_x ;
        int tmp_y = neighbor_y ;
        if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y))
            neighbors.push_back(Node(neighbor_x, neighbor_y));
        if (dx != 0 && dy != 0){
            for(int i = 0; i < 2; i++){
                tmp_x = tmp_x - dx;
                tmp_y = tmp_y - dy;
                if (is_valid_neighbor(map, width, height, neighbor_x, tmp_y))
                    neighbors.push_back(Node(neighbor_x, tmp_y));
                if (is_valid_neighbor(map, width, height, tmp_x, neighbor_y))
                    neighbors.push_back(Node(tmp_x, neighbor_y));
            }
        }
        else {
            if(dx != 0 && dy == 0){
                int tmp_y1 = neighbor_y + 1;
                int tmp_y2 = neighbor_y - 1;
                if (is_valid_neighbor(map, width, height, neighbor_x, tmp_y1))
                    neighbors.push_back(Node(neighbor_x, tmp_y1));
                if (is_valid_neighbor(map, width, height, neighbor_x, tmp_y2))
                    neighbors.push_back(Node(neighbor_x, tmp_y2));
            }
            else if(dx == 0 && dy != 0){
                int tmp_x1 = neighbor_x + 1;
                int tmp_x2 = neighbor_x - 1;
                if (is_valid_neighbor(map, width, height, tmp_x1, neighbor_y))
                    neighbors.push_back(Node(tmp_x1, neighbor_y));
                if (is_valid_neighbor(map, width, height, tmp_x2, neighbor_y))
                    neighbors.push_back(Node(tmp_x2, neighbor_y));
            }
        }
        if (dx == 0 && dy == 0){
            for (int i = 0; i < method; i++){
                neighbor_x = cur_x + offsets[i][0];
                neighbor_y = cur_y + offsets[i][1];
                if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y))
                    neighbors.push_back(Node(neighbor_x, neighbor_y));
            }
        }
    }
    else if(method == 16 || method == 32){
        neighbor_x = cur_x + dx;
        neighbor_y = cur_y + dy;
        int tmp_x = neighbor_x;
        int tmp_y = neighbor_y;
        if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y))
            neighbors.push_back(Node(neighbor_x, neighbor_y));
        if (dx != 0 && dy != 0) {
            for (int i = 0; i < 2; i++) {
                tmp_x = tmp_x - dx;
                tmp_y = tmp_y - dy;
                if (is_valid_neighbor(map, width, height, neighbor_x, tmp_y))
                    neighbors.push_back(Node(neighbor_x, tmp_y));
                if (is_valid_neighbor(map, width, height, tmp_x, neighbor_y))
                    neighbors.push_back(Node(tmp_x, neighbor_y));
            }
        } else {
            if (dx != 0 && dy == 0) {
                int tmp_y1 = neighbor_y + 1;
                int tmp_y2 = neighbor_y - 1;
                if (is_valid_neighbor(map, width, height, neighbor_x, tmp_y1))
                    neighbors.push_back(Node(neighbor_x, tmp_y));
                if (is_valid_neighbor(map, width, height, neighbor_x, tmp_y2))
                    neighbors.push_back(Node(tmp_x, neighbor_y));
            } else {
                int tmp_x1 = neighbor_x + 1;
                int tmp_x2 = neighbor_x - 1;
                if (is_valid_neighbor(map, width, height, tmp_x1, neighbor_y))
                    neighbors.push_back(Node(neighbor_x, tmp_y));
                if (is_valid_neighbor(map, width, height, tmp_x1, neighbor_y))
                    neighbors.push_back(Node(tmp_x, neighbor_y));
            }
        }
        for (int i = 8; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y) && is_valid_move(map, width, height, Node(cur->x, cur->y), Node(neighbor_x, neighbor_y)))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
    }
    return neighbors;
}

std::pair<std::vector<Node>, double> AStarByLink::construct_path(std::shared_ptr<ANode> node) {

    double cost = -1.0;
    std::vector<Node > path;
    std::shared_ptr<ANode> cur = std::move(node);
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
            cur = cur->prev.lock();
        }
    }

    return std::make_pair(path, cost);
}

bool AStarByLink::is_valid_neighbor(std::vector<std::vector<int>> &map, int width, int height, int x, int y) {
    if( x >= 0 && x < width && y >= 0 && y < height && map[x][y] == 0) // map [y][x]
        return true;
    return false;
}

bool
AStarByLink::is_valid_move(std::vector<std::vector<int>> &map, int width, int height, const Node &sp, const Node &ep) {
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
            if (map[i][x1]) // obstacle
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
            if (map[y1][i])    //obstacle
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
    if (fabs((slope - 0.0)) > 1e-3){
        for (double xi = x1; xi < x2; ){
            int tmp_x = round(xi);
            int tmp_y = round(slope*(xi - x1) + y1);
            if (!is_valid_neighbor(map, width, height, tmp_x, tmp_y))    //obstacle
                return false;
            xi = xi + 0.1;
        }
    }
    return true;
}

bool AStarByLink::is_valid_move_new(std::vector<std::vector<int>> &map, Node cur, int width, int height, int i) {
    int size = new_offsets[i][8];
    int cur_x = cur.x;
    int cur_y = cur.y;
    int neighbor_x = 0;
    int neighbor_y = 0;
    for (int j = 0; j < size; j++){
        neighbor_x = cur_x + new_offsets[i][2 * j];
        neighbor_y = cur_x + new_offsets[i][2 * j + 1];
        if (!is_valid_neighbor(map, width, height, neighbor_x, neighbor_y))
            return false;
    }
    return true;
}

