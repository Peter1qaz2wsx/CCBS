#include <iostream>
#include <fstream>
#include "map.h"
#include "task.h"
#include "cbs.h"
#include "xml_logger.h"

int main(int argc, const char *argv[])
{
    if(argc > 2)
    {
        Config config;
        if(argc > 3)
            config.getConfig(argv[3]);
        Map map = Map(config.agent_size, config.connectdness); // 初始化map结构
        map.get_map(argv[1]); // 通过传入的参数，完成地图的构造
        Task task;
        task.get_task(argv[2]); // 通过传入发task.xml文件完成egents 初始位置和目标位置信息的输入
        if(map.is_roadmap()) // 判断地图的类型，根据地图的类型构造task的类型
            task.make_ij(map);
        else
            task.make_ids(map.get_width());
        CBS cbs;
        Solution solution = cbs.find_solution(map, task, config);  
        // while (!solution.found) {
        //     config.agent_size = 2 * config.agent_size;
        //     solution = cbs.find_solution(map, task, config);  
        // }
        // 应用CBS求解MAPF的问题，输入的参数是“地图”“任务”“参数”
        XML_logger logger;
        auto found = solution.found?"true":"false";
        std::cout<< "Soulution found: " << found << "\nRuntime: "<<solution.time.count() << "\nMakespan: " << solution.makespan << "\nFlowtime: " << solution.flowtime<< "\nInitial Cost: "<<solution.init_cost<< "\nCollision Checking Time: " << solution.check_time
             << "\nHL expanded: " << solution.high_level_expanded << "\nLL searches: " << solution.low_level_expansions << "\nLL expanded(avg): " << solution.low_level_expanded << std::endl;

        logger.get_log(argv[2]);  // 获取输出结果的路径
        logger.write_to_log_summary(solution);
        logger.write_to_log_path(solution, map);
        logger.save_log();
        // 将CBS求解的结果写在“task_log.xml”中
    }
    else
    {
        std::cout<<"Error! Not enough input parameters are specified!\n";
    }
    return 0;
}
