
#include <iostream>
#include "../include/test_gridmap.h"
#include "../include/GridMap.h"
#include "../include/AStar.h"
#include "../include/DataLoad.h"
#include "../include/AStarByList.h"

#include <iostream>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

using namespace std;
void test_path_plan();

int main() {
    std::cout << "Hello, World!" << std::endl;
    test_path_plan();
    return 0;
}

void test_path_plan(){
    const char* file_name = "../map_txt/Random_100x100_.3.txt";
    const char* input_name = "../input/Random_100x100_.3.txt";
    GridMap gridMap(file_name);
    Node Target(1, 1);

    std::vector<std::vector<int> > start_point_list;

    load_data(input_name, start_point_list);

    int methods[1] = { 8};

    std::string output_filename = R"(../output/test_log.txt)";
    std::fstream test_output(output_filename, std::ios_base::out | std::ios_base::app);

    time_t nowtime = time(NULL);
    tm* p = localtime(&nowtime);
    std::stringstream now_string;
    now_string << p->tm_year+1900 << " " << p->tm_mon+1 << " " << p->tm_mday <<
               " " << p->tm_hour << " " << p->tm_min <<" " << p->tm_sec;

    test_output << "\n" << now_string.str() << std::endl;
    test_output << "A Star !" << std::endl;
    test_output << file_name << std::endl;

    std::stringstream record_string;

    AStarbyList aStarbyList(file_name);
//    AStar astar (gridMap.self_map);
    for (int method: methods){
        for(auto cor: start_point_list){
            Node Start(cor[0], cor[1]);
            auto before_time = std::chrono::high_resolution_clock::now();
            std::pair<std::vector<Node >, double> result = aStarbyList.run(file_name, Start, Target, method);
            //std::pair<std::vector<Node >, double> result = astar.PrintAStarPath({Start.x, Start.y}, {Target.x, Target.y});
            auto after_time = std::chrono::high_resolution_clock::now();
            double duration_millsecond = std::chrono::duration<double, std::milli>(after_time-before_time).count();

            std::stringstream output_string;
            output_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;
            record_string <<  "running time cost: " << duration_millsecond << " ms \t" << "path length: " << result.second << " method: " << method << std::endl;

            std::reverse(result.first.begin(), result.first.end());
            output_string << "path: " << " " << "[" << Start.x << ", " << Start.y << "], ";
            for(const auto& i: result.first){
                output_string << "[" << i.x << ", " << i.y << "], ";
            }
            test_output << output_string.str() << std::endl;
            std::vector<Node > mp;
            result.first.clear();
            result.first.shrink_to_fit();
        }
    }

    test_output << "\n" << record_string.str() << std::endl;
    test_output << "\n" << std::endl;
    test_output.close();

    now_string.clear();
    now_string.str("");
    record_string.clear();
    record_string.str("");

    malloc_trim(0);
}

//int main() {
//    vector<int> temp_line;
//    vector<vector<int>> Vec_Dti;
//    string line;
//    ifstream in("../map_txt/Random_100x100_.3.txt");  //读入文件
//    regex pat_regex("[[:digit:]]+");  //匹配原则，这里代表一个或多个数字
//
//    while(getline(in, line)) {  //按行读取
//        for (sregex_iterator it(line.begin(), line.end(), pat_regex), end_it; it != end_it; ++it) {  //表达式匹配，匹配一行中所有满足条件的字符
////            cout << it->str() << " ";  //输出匹配成功的数据
//            temp_line.push_back(stoi(it->str()));  //将数据转化为int型并存入一维vector中
//        }
//        Vec_Dti.push_back(temp_line);  //保存所有数据
//        temp_line.clear();
//    }
//
//    for(auto i : Vec_Dti) {  //输出存入vector后的数据
//        for(auto j : i) {
//            cout << j << " ";
//        }
//        cout << endl;
//    }
//    std::cout << "gridmap size: " << Vec_Dti.size()  << "gridmap[0] size" << Vec_Dti[0].size() << std::endl;
//    return 0;
//}




