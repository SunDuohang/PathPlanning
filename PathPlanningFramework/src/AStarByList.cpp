/**
  ******************************************************************************
  * @file           : AStarByList.cpp
  * @author         : sunx
  * @brief          : None
  * @attention      : None
  * @date           : 24-3-7
  ******************************************************************************
  */

#include "../include/AStarByList.h"

/**
 * @brief 构造函数
 * @param filename
 */
AStarbyList::AStarbyList(const char* filename){
    gridMap1.set_GridMap(filename);
    map_height = gridMap1.self_height;
    map_width = gridMap1.self_width;
}

std::pair<std::vector<Node >, double> AStarbyList::run(const char* filename,Node& start_point, Node& target_point, int method) {
    GridMap gridMap(filename);
    int width = gridMap.self_width;
    int height = gridMap.self_height;

    std::vector<Record_Item > close_list;
    std::vector<Record_Item > open_list;

    double g_value = 0.0;
    double h_value = Octi_heuristic(start_point.x, start_point.y, target_point.x, target_point.y);
    double f_value = g_value + h_value;
    Record_Item start_info(start_point, g_value, f_value, start_point, 0);
    open_list.push_back(start_info);

    std::make_heap(open_list.begin(), open_list.end(), greater1());
    while(!open_list.empty()){
        Record_Item open_item = open_list[0];
        std::pop_heap(open_list.begin(), open_list.end(), greater1());
        open_list.pop_back();
        Node cur_node = open_item.self_cur_cor;
        Node parent_node = open_item.self_parent_cor;

        int parentnode_index = is_in_list(close_list, parent_node);
        if (parentnode_index == -1){
            Record_Item close_item(open_item, 0);
            close_list.push_back(close_item);
        }
        else{
            Record_Item close_item(open_item, parentnode_index);
            close_list.push_back(close_item);
        }

        if (target_point.x == cur_node.x && target_point.y == cur_node.y){
            auto result = construct_path(close_list);

            //该部分可以删除
//            std::string output_filename = R"(../output/record_log.txt)";
//            std::fstream record_output(output_filename, std::ios_base::out | std::ios_base::app);
//            std::stringstream out_string;
//            out_string << "strat_point-target_point: [" << start_point.x << ", " << start_point.y << "]-[" << target_point.x << ", " << target_point.y << "]" << "\t";
//            out_string << "open_list size: " << open_list.size() << "\t close_list size: " << close_list.size() << "\t" ;
//            out_string << "path_size: " << result.first.size() << "\t";
//            record_output << out_string.str() << std::endl;
//            out_string.clear();
//            out_string.str("");
//            record_output.close();

            open_list.clear();
            open_list.shrink_to_fit();
            close_list.clear();
            close_list.shrink_to_fit();
            return result;
        }

        std::vector<Node > neighbors = get_neighbors(gridMap.self_map, width, height, cur_node, method);
        //std::vector<Node > neighbors = get_neighbors_new(gridMap.self_map, width, height, cur_node, method);


//        int dx = cur_node.x - parent_node.x;
//        int dy = cur_node.y - parent_node.y;
//
//        std::vector<Node > neighbors = get_neighbors(map, width, height, cur_node, dx, dy, method);

        for(Node node: neighbors){
            if (is_in_list(close_list, node) != -1)
                continue;
            else{
                int open_list_index = is_in_list(open_list, node);
                if (open_list_index != -1){
                    double cur_g_value = open_item.self_g_value + distance(cur_node.x, cur_node.y, node.x, node.y);
                    if (cur_g_value < open_list[open_list_index].self_g_value){
                        double cur_h_value = Octi_heuristic(node.x, node.y, target_point.x, target_point.y);
                        open_list.erase(open_list.begin()+open_list_index);
                        Record_Item node_info(node, cur_g_value,  (cur_g_value + cur_h_value), cur_node, open_list.size());
                        open_list.push_back(node_info);
                        std::make_heap(open_list.begin(), open_list.end(), greater1());

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
                    double cur_h_value = Octi_heuristic(node.x, node.y, target_point.x, target_point.y);
                    Record_Item node_info(node, cur_g_value,  (cur_g_value + cur_h_value), cur_node, open_list.size());
                    open_list.push_back(node_info);
                    std::make_heap(open_list.begin(), open_list.end(), greater1());

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
    std::vector<Node > path;
    double cost = -1.0;

//    std::string output_filename = R"(../output/record_log.txt)";
//    std::fstream record_output(output_filename, std::ios_base::out | std::ios_base::app);
//    std::stringstream out_string;
//    out_string << "strat_point-target_point: [" << start_point.x << ", " << start_point.y << "]-[" << target_point.x << ", " << target_point.y << "]" << "\t ";
//    out_string << "open_list size: " << open_list.size() << "\t close_list size: " << close_list.size() << "\t" ;
//    out_string << "path_size: " << path.size() << std::endl ;
//    record_output << out_string.str() << std::endl;
//    record_output.close();

    open_list.clear();
    open_list.shrink_to_fit();
    close_list.clear();
    close_list.shrink_to_fit();
    return std::make_pair(path, cost);
}

std::pair<std::vector<Node>, double>AStarbyList::run(const GridMap &gridMap, const Node &start_point, const Node &target_point, int method) {
    int width = gridMap.self_width;
    int height = gridMap.self_height;

    std::vector<Record_Item > close_list;
    std::vector<Record_Item > open_list;

    // 该部分不可以删除
    double g_value = 0.0;
    double h_value = Octi_heuristic(start_point.x, start_point.y, target_point.x, target_point.y);
    double f_value = g_value + h_value;
    Record_Item start_info(start_point, g_value, f_value, start_point, 0);
    open_list.push_back(start_info);

    std::make_heap(open_list.begin(), open_list.end(), greater1());
    while(!open_list.empty()){
        Record_Item open_item = open_list[0];
        std::pop_heap(open_list.begin(), open_list.end(), greater1());
        open_list.pop_back();
        Node cur_node = open_item.self_cur_cor;
        Node parent_node = open_item.self_parent_cor;

        int parentnode_index = is_in_list(close_list, parent_node);
        if (parentnode_index == -1){
            Record_Item close_item(open_item, 0);
            close_list.push_back(close_item);

//            //该部分可以删除
//            closelist_out_string << "closelist: \n" << "[" << close_item.self_cur_cor.x << ", " << close_item.self_cur_cor.y << "], " <<
//                                 close_item.self_g_value << ", " << close_item.self_f_value << ", [" << close_item.self_parent_cor.x << ", " << close_item.self_parent_cor.y << "]; \n" ;
//            closelist_output << closelist_out_string.str() ;
//            closelist_out_string.clear();
//            closelist_out_string.str("");
        }
        else{
            Record_Item close_item(open_item, parentnode_index);
            close_list.push_back(close_item);

//            //该部分可以删除
//            closelist_out_string << "closelist: \n" << "[" << close_item.self_cur_cor.x << ", " << close_item.self_cur_cor.y << "], " <<
//                                close_item.self_g_value << ", " << close_item.self_f_value << ", [" << close_item.self_parent_cor.x << ", " << close_item.self_parent_cor.y << "]; \n" ;
//            closelist_output << closelist_out_string.str() ;
//            closelist_out_string.clear();
//            closelist_out_string.str("");
        }

        if (target_point.x == cur_node.x && target_point.y == cur_node.y){
            auto result = construct_path(close_list);

            //该部分可以删除
//            std::string output_filename = R"(../output/record_log.txt)";
//            std::fstream record_output(output_filename, std::ios_base::out | std::ios_base::app);
//            std::stringstream out_string;
//            out_string << "strat_point-target_point: [" << start_point.x << ", " << start_point.y << "]-[" << target_point.x << ", " << target_point.y << "]" << "\t";
//            out_string << "open_list size: " << open_list.size() << "\t close_list size: " << close_list.size() << "\t" ;
//            out_string << "path_size: " << result.first.size() << "\t";
//            record_output << out_string.str() << std::endl;
//            out_string.clear();
//            out_string.str("");
//            record_output.close();
//
//            open_list.clear();
//            open_list.shrink_to_fit();
//            close_list.clear();
//            close_list.shrink_to_fit();
            return result;
        }

        std::vector<Node > neighbors = get_neighbors_new(gridMap.self_map, width, height, cur_node, method);
        for(Node node: neighbors){
            if (is_in_list(close_list, node) != -1)
                continue;
            else{
                int open_list_index = is_in_list(open_list, node);
                if (open_list_index != -1){
                    double cur_g_value = open_item.self_g_value + distance(cur_node.x, cur_node.y, node.x, node.y);
                    if (cur_g_value < open_list[open_list_index].self_g_value){
                        double cur_h_value = Octi_heuristic(node.x, node.y, target_point.x, target_point.y);
                        open_list.erase(open_list.begin()+open_list_index);
                        Record_Item node_info(node, cur_g_value,  (cur_g_value + cur_h_value), cur_node, open_list.size());
                        open_list.push_back(node_info);
                        std::make_heap(open_list.begin(), open_list.end(), greater1());

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
                    double cur_h_value = Octi_heuristic(node.x, node.y, target_point.x, target_point.y);
                    Record_Item node_info(node, cur_g_value,  (cur_g_value + cur_h_value), cur_node, open_list.size());
                    open_list.push_back(node_info);
                    std::make_heap(open_list.begin(), open_list.end(), greater1());

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
    std::vector<Node > path;
    double cost = -1.0;

//    std::string output_filename = R"(../output/record_log.txt)";
//    std::fstream record_output(output_filename, std::ios_base::out | std::ios_base::app);
//    std::stringstream out_string;
//    out_string << "strat_point-target_point: [" << start_point.x << ", " << start_point.y << "]-[" << target_point.x << ", " << target_point.y << "]" << "\t ";
//    out_string << "open_list size: " << open_list.size() << "\t close_list size: " << close_list.size() << "\t" ;
//    out_string << "path_size: " << path.size() << std::endl ;
//    record_output << out_string.str() << std::endl;
//    record_output.close();

    open_list.clear();
    open_list.shrink_to_fit();
    close_list.clear();
    close_list.shrink_to_fit();

    return std::make_pair(path, cost);
}

std::vector<Node>
AStarbyList::get_neighbors(std::vector<std::vector<int>> &map, int width, int height, const Node& cur, int method) {
    std::vector<Node > neighbors;
    int cur_x = cur.x;
    int cur_y = cur.y;
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
            if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y) && is_valid_move(map, width, height, cur, Node(neighbor_x, neighbor_y)))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
    }
    return neighbors;}

/**
 * @brief 拒绝重复访问 来加快路径规划。即当前节点不会重复访问父节点已访问变量
 * @param map
 * @param width
 * @param height
 * @param cur
 * @param dx
 * @param dy
 * @param method
 * @return
 */
std::vector<Node>
AStarbyList::get_neighbors(std::vector<std::vector<int>> &map, int width, int height, const Node &cur, int dx, int dy,
                           int method) {
    std::vector<Node > neighbors;
    int cur_x = cur.x;
    int cur_y = cur.y;
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
            if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y) && is_valid_move(map, width, height, cur, Node(neighbor_x, neighbor_y)))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
    }
    return neighbors;
}

std::vector<Node> AStarbyList::get_neighbors_new(std::vector<std::vector<int>> map, int width, int height, const Node &cur,
                               int method) {
    std::vector<Node > neighbors;
    int cur_x = cur.x;
    int cur_y = cur.y;
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
            int tmp_x = 0;
            int tmp_y = 0;
            if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y)){
                int size = new_offsets[i][8];
                int flag = 1;
                for (int j = 0; j < size; j++){
                    tmp_x = cur_x + new_offsets[i][2 * j];
                    tmp_y = cur_y + new_offsets[i][2 * j + 1];
                    if (!is_valid_neighbor(map, width, height, tmp_x, tmp_y)){
                        flag = 0;
                        break;
                    }
                }
                if (flag){
                    neighbors.push_back(Node(neighbor_x, neighbor_y));
                }
            }
        }
    }
    return neighbors;
}

std::vector<Node> AStarbyList::get_neighbors_oritanted(std::vector<std::vector<int>> &map, int width, int height, Node& cur,
                                     const short *flags, int lens) {
    std::vector<Node > neighbors;
    int cur_x = cur.x;
    int cur_y = cur.y;
    int neighbor_x = 0;
    int neighbor_y = 0;
    for (int i = 0; i < 8; i++){
        neighbor_x = cur_x + offsets[i][0];
        neighbor_y = cur_y + offsets[i][1];
        if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y))
            neighbors.push_back(Node(neighbor_x, neighbor_y));
    }

    for (int i = 0; i < lens; i++){
        short index = flags[i];
        neighbor_x = cur_x + change_offsets[index][0];
        neighbor_y = cur_y + change_offsets[index][1];
        int tmp_x = 0;
        int tmp_y = 0;
        if (is_valid_neighbor(map, width, height, neighbor_x, neighbor_y)){
            int size = change_offsets_obs[index][8];
            int flag = 1;
            for (int j = 0; j < size; j++){
                tmp_x = cur_x + change_offsets_obs[index][2 * j];
                tmp_y = cur_y + change_offsets_obs[index][2 * j + 1];
                if (!is_valid_neighbor(map, width, height, tmp_x, tmp_y)){
                    flag = 0;
                    break;
                }
            }
            if (flag){
                neighbors.push_back(Node(neighbor_x, neighbor_y));
            }
        }
    }
    return neighbors;
}

bool AStarbyList::is_valid_neighbor(std::vector<std::vector<int>> &map, int width, int height, int x, int y) {
    if( x >= 0 && x < width && y >= 0 && y < height && map[x][y] == 0) // map [y][x]
        return true;
    return false;
}

bool
AStarbyList::is_valid_move(std::vector<std::vector<int>> &map, int width, int height, const Node &sp, const Node &ep) {
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
    return true;}

bool AStarbyList::is_valid_move_new(std::vector<std::vector<int>> &map, Node& cur, int width, int height, int i) {
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

double AStarbyList::Euli_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor) {
    double h = sqrt(pow((x1_cor - x2_cor), 2) + pow((y1_cor - y2_cor), 2));
    return h;
}

double AStarbyList::Manh_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor) {
    return std::fabs(x2_cor - x1_cor) + std::fabs(y2_cor - y1_cor);
}

double AStarbyList::Cheb_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor) {
    return std::fmax(std::fabs(x2_cor - x1_cor), std::fabs(y2_cor - y1_cor));
}

double AStarbyList::Octi_heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor) {
    double h = sqrt(2)*std::min(std::abs(x2_cor - x1_cor), std::abs(y2_cor - y1_cor));
    double s = std::abs(std::abs(x2_cor - x1_cor) - std::abs(y2_cor - y1_cor));
    return h+s;
}

double AStarbyList::distance(int x1_cor, int y1_cor, int x2_cor, int y2_cor) {
    double d = sqrt(pow((x1_cor - x2_cor), 2) + pow((y1_cor - y2_cor), 2));
    return d;
}

std::vector<Node> AStarbyList::get_lines(Node& sp, Node& ep) {
    std::vector<Node > line;
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
            line.push_back(Node(x1, i));
        }
    }
    else if(y1 == y2){
        if(x1 > x2){
            int tmp = x1;
            x1 = x2;
            x2 = tmp;
        }
        for(int i = x1; i < x2; i++){
            line.push_back(Node(i, y1));
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
            if (!is_in_line(line, Node(tmp_x, tmp_y)))
                line.push_back(Node(tmp_x, tmp_y));
            xi = xi + 0.1;
        }
    }
    return line;
}

bool AStarbyList::is_in_line(std::vector<Node>& line, Node p) {
    for (const Node& i: line){
        if (i.x == p.x && i.y == p.y)
            return true;
    }
    return false;
}

std::pair<std::vector<Node>, double> AStarbyList::construct_path(std::vector<Record_Item>& close_list) {
    std::vector<Node > path;
    double cost = 0.0;
    Record_Item item = close_list.back();
    cost = item.self_g_value;
    int parent_index = 0;
    while(!(item.self_cur_cor.x == item.self_parent_cor.x && item.self_cur_cor.y == item.self_parent_cor.y)){
        parent_index = (item).self_parent_index;
        Node node((item).self_cur_cor);
        path.push_back(node);
        item = close_list.at(parent_index);
    }
    std::pair<std::vector<Node >, double> result = std::make_pair(path, cost);
    path.clear();
    path.shrink_to_fit();
    return result;
}

int AStarbyList::is_in_list(std::vector<Record_Item> &list, const Node& cur_node) {
    for(int i = 0; i < list.size(); i++){
        Node tmp_node = list[i].self_cur_cor;
        if (cur_node.x == tmp_node.x && cur_node.y == tmp_node.y)
            return i;
    }
    return -1;
}

int AStarbyList::get_target_angle_seg(const Node &sp, const Node &ep) {
    double angle_rad = atan2((double)(ep.y -sp.y), (double)(ep.x - sp.x));
    double pi = M_PI;
    double angle_deg = (angle_rad / pi) * 180 + 180;
    int seg = (int)(angle_deg / 45);
    return (seg%8);}

void AStarbyList::set_changeable_offsets(const Node &sp, const Node &ep) {

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















