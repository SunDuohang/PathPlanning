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

#include "../include/AStar.h"

AStar::AStar(const char* filename){

    get_map_size(filename, self_width, self_height);
    self_map = new int* [self_height];
    for(int i = 0; i < self_height; i++)
        self_map[i] = new int [self_width];
    load_map(filename, self_map, self_width, self_height);

}


AStar::~AStar(){
    for(int i = 0; i < self_height; i++){
        delete []self_map[i];
    }
    delete []self_map;
    malloc_trim(0);
}


std::pair<std::vector<Node >, double> AStar::run(const char* filename,Node start_point, Node target_point, int method){
    int** map;
    int width = 0;
    int height = 0;
    get_map_size(filename, width, height);
    map = new int * [width];
    for(int i = 0; i < height; i++)
        map[i] = new int [width];
    load_map(filename, map, width, height);
    std::vector<Record_Item > close_list;
    std::vector<Record_Item > open_list;

//    // 该部分可以删除
//    std::fstream openlist_output(R"(../output/open_list.txt)", std::ios_base::out | std::ios_base::app);
//    std::fstream closelist_output(R"(../output/close_list.txt)", std::ios_base::out | std::ios_base::app);
//    std::stringstream openlist_out_string;
//    std::stringstream closelist_out_string;
//    openlist_out_string << "\n" << filename ;
//    openlist_output << openlist_out_string.str() << std::endl;
//    closelist_output << openlist_out_string.str() << std::endl;
//    openlist_out_string.clear();
//    openlist_out_string.str("");

    // 该部分不可以删除
    double g_value = 0.0;
    double h_value = heuristic(start_point.x, start_point.y, target_point.x, target_point.y);
    double f_value = g_value + h_value;
    Record_Item start_info(start_point, g_value, f_value, start_point, 0);
    open_list.push_back(start_info);

//    //该部分可以删除
//    openlist_out_string << "openlist: \n" << "[" << start_info.self_cur_cor.x << ", " << start_info.self_cur_cor.y << "], " <<
//    start_info.self_g_value << ", " << start_info.self_f_value << ", [" << start_info.self_parent_cor.x << ", " << start_info.self_parent_cor.y << "]; \n";
//    openlist_output << openlist_out_string.str() ;
//    openlist_out_string.clear();
//    openlist_out_string.str("");

    // 创建为最小堆
    std::make_heap(open_list.begin(), open_list.end(), greater1());
    while(!open_list.empty()){
        Record_Item open_item = open_list[0];
        std::pop_heap(open_list.begin(), open_list.end(), greater1());
        open_list.pop_back();
        Node cur_node = open_item.self_cur_cor;
        Node parent_node = open_item.self_parent_cor;
//        g_value = open_item.self_g_value;
//        h_value = heuristic(cur_node.x, cur_node.y, target_point.x, target_point.y);
//        f_value = g_value + h_value;
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
            std::string output_filename = R"(../output/record_log.txt)";
            std::fstream record_output(output_filename, std::ios_base::out | std::ios_base::app);
            std::stringstream out_string;
            out_string << filename << "\t";
            out_string << "strat_point-target_point: [" << start_point.x << ", " << start_point.y << "]-[" << target_point.x << ", " << target_point.y << "]" << "\t";
            out_string << "open_list size: " << open_list.size() << "\t close_list size: " << close_list.size() << "\t" ;
            out_string << "path_size: " << result.first.size() << "\t";
            record_output << out_string.str() << std::endl;
            out_string.clear();
            out_string.str("");
            record_output.close();
//            openlist_output.close();

            for(int i = 0; i < width; i++)
                delete []map[i];
            delete []map;

            open_list.clear();
            open_list.shrink_to_fit();
            close_list.clear();
            close_list.shrink_to_fit();
            return result;
        }

        std::vector<Node > neighbors = get_neighbors_new(map, width, height, cur_node, method);
        for(Node node: neighbors){
            if (is_in_list(close_list, node) != -1)
                continue;
            else{
                int open_list_index = is_in_list(open_list, node);
                if (open_list_index != -1){
                    double cur_g_value = open_item.self_g_value + distance(cur_node.x, cur_node.y, node.x, node.y);
                    if (cur_g_value < open_list[open_list_index].self_g_value){
                        double cur_h_value = heuristic(node.x, node.y, target_point.x, target_point.y);
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
                    double cur_h_value = heuristic(node.x, node.y, target_point.x, target_point.y);
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
    for(int i = 0; i < height; i++)
        delete []map[i];
    delete []map;

    return std::make_pair(path, cost);
}


std::vector<Node > AStar::get_neighbors(int** map, int map_xsize, int map_ysize, Node cur, int method){
    std::vector<Node > neighbors;
    int cur_x = cur.x;
    int cur_y = cur.y;
    int neighbor_x = 0;
    int neighbor_y = 0;
    if (method == 4 || method == 8){
        for (int i = 0; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
    }
    else if(method == 16 || method == 32){
        for (int i = 0; i < 8; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
        for (int i = 8; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map) && is_valid_move(map, cur, Node(neighbor_x, neighbor_y), map_xsize, map_ysize))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
    }
    return neighbors;
};


std::vector<Node > AStar::get_neighbors_new(int** map, int map_xsize, int map_ysize, const Node& cur, int method){
    std::vector<Node > neighbors;
    int cur_x = cur.x;
    int cur_y = cur.y;
    int neighbor_x = 0;
    int neighbor_y = 0;
    if (method == 4 || method == 8){
        for (int i = 0; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
    }
    else if(method == 16 || method == 32){
        for (int i = 0; i < 8; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map))
                neighbors.push_back(Node(neighbor_x, neighbor_y));
        }
        for (int i = 8; i < method; i++){
            neighbor_x = cur_x + offsets[i][0];
            neighbor_y = cur_y + offsets[i][1];
            int tmp_x = 0;
            int tmp_y = 0;
            if (is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map)){
                int size = new_offsets[i][8];
                int flag = 1;
                for (int j = 0; j < size; j++){
                    tmp_x = cur_x + new_offsets[i][2 * j];
                    tmp_y = cur_y + new_offsets[i][2 * j + 1];
                    if (!is_valid_neighbor(tmp_x, tmp_y, map_xsize, map_ysize, map)){
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

bool AStar::is_valid_neighbor(int x, int y, int map_xsize, int map_ysize, int** map) {
    if( x >= 0 && x < map_xsize && y >= 0 && y < map_ysize && map[x][y] == 0) // map [y][x]
        return true;
    return false;
}


double AStar::heuristic(int x1_cor, int y1_cor, int x2_cor, int y2_cor){
    double h = sqrt(pow((x1_cor - x2_cor), 2) + pow((y1_cor - y2_cor), 2));
    return h;
};


double AStar::distance(int x1_cor, int y1_cor, int x2_cor, int y2_cor){
    double d = sqrt(pow((x1_cor - x2_cor), 2) + pow((y1_cor - y2_cor), 2));
    return d;
};

/**
 * @brief get point on line
 * @tparam T
 * @param sp
 * @param ep
 * @return
 */

std::vector<Node > AStar::get_lines(Node sp, Node ep){
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
};


bool AStar::is_in_line(std::vector<Node> line, Node p) {
    for (Node i: line){
        if (i.x == p.x && i.y == p.y)
            return true;
    }
    return false;
}


bool AStar::is_valid_move(int **map, Node sp, Node ep, int map_xsize, int map_ysize) {
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
            if (!is_valid_neighbor(tmp_x, tmp_y, map_xsize, map_ysize, map))    //obstacle
                return false;
            xi = xi + 0.1;
        }
    }
    return true;
}


bool AStar::is_valid_move_new(int **map, Node cur, int map_xsize, int map_ysize, int i){
    int size = new_offsets[i][8];
    int cur_x = cur.x;
    int cur_y = cur.y;
    int neighbor_x = 0;
    int neighbor_y = 0;
    for (int j = 0; j < size; j++){
        neighbor_x = cur_x + new_offsets[i][2 * j];
        neighbor_y = cur_y + new_offsets[i][2 * j + 1];
        if (!is_valid_neighbor(neighbor_x, neighbor_y, map_xsize, map_ysize, map)){
            return false;
        }
    }
    return true;
}

/*template <class T1, class T2>
bool Astar<T1, T2>::is_valid_move_new(T1 **map, Node<T1> sp, Node<T1> ep) {
    int x1 = (int) sp.x;
    int x2 = (int) ep.x;
    int y1 = (int) sp.y;
    int y2 = (int) ep.y;
    double dx = x1 - x2;
    double dy = y1 - y2;
    double f = 0;
    int sx = 1;
    int sy = 1;
    if (dy < 0.0){
        dy = -dy;
        sy = -1;
    }
    if (dx < 0.0){
        dx = -dx;
        sx = -1;
    }
    if (dx >= dy){
        while(x1 != x2){
            f = f + dy;
            if (f >= dx){
                if (map[x1 + ((sx-1) / 2)][y1 + ((sy-1) / 2)])
                    return false;
                y1 = y1 + sy;
                f = f - dx;
            }
            if ((f != 0.0) && map[x1+((sx-1) / 2)][y1+((sy-1) / 2)])
                return false;
            if ((dy == 0.0) && map[x1+((sx-1) / 2)][y1] && map[x1+((sx-1) / 2)][y1-1])
                return false;
            x1 = x1 + sx;
        }
    } else{
        while (y1 != y2){
            f = f + dx;
            if (f >= dy){
                if (map[x1 + ((sx-1) / 2)][y1 + ((sy-1) / 2)])
                    return false;
                x1 = x1 + sx;
                f = f - dy;
            }
            if ((f != 0) && map[x1 + ((sx-1) / 2)][y1 + ((sy-1) / 2)])
                return false;
            if ((dx == 0.0) && map[x1][y1 + (sy-1) / 2] && map[x1 - 1][y1 + ((sy-1) / 2)])
                return false;
            y1 = y1 + sy;
        }
    }
    return true;
}*/


int AStar::is_in_list(std::vector<Record_Item> list, Node cur_node) {
    for(int i = 0; i < list.size(); i++){
        Node tmp_node = list[i].self_cur_cor;
        if (cur_node.x == tmp_node.x && cur_node.y == tmp_node.y)
            return i;
    }
    return -1;
}


bool AStar::is_in_open_list(std::vector<Open_list_Item> open_list, Node cur_node) {
    for (Open_list_Item item: open_list){
        Node tmp_node = item.self_cur_cor;
    }
    return false;
}

std::pair<std::vector<Node >, double> AStar::construct_path(std::vector<Record_Item> close_list) {
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

std::pair<std::vector<Node>, double> AStar::run(const GridMap& gridMap, const Node& start_point, const Node& target_point, int method) {

    int width = gridMap.self_width;
    int height = gridMap.self_height;

    std::vector<Record_Item > close_list;
    std::vector<Record_Item > open_list;

//    // 该部分可以删除
//    std::fstream openlist_output(R"(../output/open_list.txt)", std::ios_base::out | std::ios_base::app);
//    std::fstream closelist_output(R"(../output/close_list.txt)", std::ios_base::out | std::ios_base::app);
//    std::stringstream openlist_out_string;
//    std::stringstream closelist_out_string;
//    openlist_out_string << "\n" << filename ;
//    openlist_output << openlist_out_string.str() << std::endl;
//    closelist_output << openlist_out_string.str() << std::endl;
//    openlist_out_string.clear();
//    openlist_out_string.str("");

    // 该部分不可以删除
    double g_value = 0.0;
    double h_value = heuristic(start_point.x, start_point.y, target_point.x, target_point.y);
    double f_value = g_value + h_value;
    Record_Item start_info(start_point, g_value, f_value, start_point, 0);
    open_list.push_back(start_info);

//    //该部分可以删除
//    openlist_out_string << "openlist: \n" << "[" << start_info.self_cur_cor.x << ", " << start_info.self_cur_cor.y << "], " <<
//    start_info.self_g_value << ", " << start_info.self_f_value << ", [" << start_info.self_parent_cor.x << ", " << start_info.self_parent_cor.y << "]; \n";
//    openlist_output << openlist_out_string.str() ;
//    openlist_out_string.clear();
//    openlist_out_string.str("");

    // 创建为最大堆
    std::make_heap(open_list.begin(), open_list.end(), greater1());
    while(!open_list.empty()){
        Record_Item open_item = open_list[0];
        std::pop_heap(open_list.begin(), open_list.end(), greater1());
        open_list.pop_back();
        Node cur_node = open_item.self_cur_cor;
        Node parent_node = open_item.self_parent_cor;
//        g_value = open_item.self_g_value;
//        h_value = heuristic(cur_node.x, cur_node.y, target_point.x, target_point.y);
//        f_value = g_value + h_value;
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

//            //该部分可以删除
//            std::string output_filename = R"(../output/record_log.txt)";
//            std::fstream record_output(output_filename, std::ios_base::out | std::ios_base::app);
//            std::stringstream out_string;
//            out_string << filename << "\t";
//            out_string << "strat_point-target_point: [" << start_point.x << ", " << start_point.y << "]-[" << target_point.x << ", " << target_point.y << "]" << "\t";
//            out_string << "open_list size: " << open_list.size() << "\t close_list size: " << close_list.size() << "\t" ;
//            out_string << "path_size: " << result.first.size() << "\t";
//            record_output << out_string.str() << std::endl;
//            out_string.clear();
//            out_string.str("");
//            record_output.close();
//            openlist_output.close();

            open_list.clear();
            open_list.shrink_to_fit();
            close_list.clear();
            close_list.shrink_to_fit();
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
                        double cur_h_value = heuristic(node.x, node.y, target_point.x, target_point.y);
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
                    double cur_h_value = heuristic(node.x, node.y, target_point.x, target_point.y);
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
