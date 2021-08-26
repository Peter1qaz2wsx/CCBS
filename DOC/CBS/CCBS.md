

# CCBS: Multi-Agent Pathfinding with Continuous Time

## CCBS 需要注意的点

- 连续 CBS (CCBS) 是对基于冲突搜索 (CBS) 算法的修改，它支持任意持续时间的动作（移动或等待）
  - 主要原因是：low-level应用的SIPP算法是一个“continuous time single-agent planning algorithm”

- CCBS 与 CBS 的不同之处在于如何定义冲突和约束
- CCBS中low-level应用的是SIPP算法
- 当前的代码支持：grid-map  road-map;　同时支持一些提升的算法：Disjoint Splitting (DS) 不相交分裂，Prioritizing conflicts (PC) 优先处理冲突，High-level heuristics 高层的启发项。
- 当前代码仓库包含的项：
  - Demo: 包含CCBS处理结果的动画
  - Examples: 包含算法需要的输入数据
  - instances: 包含带有地图和实例的档案（采用 CCBS 支持的格式），用于上述 CCBS 论文中描述的实验评估
  - ExpResults: 包含在 CCBS 算法的实验评估过程中获得的原始表格结果
  - Release: not a folder, but the tagged commits that were used to get the results for the corresponding papers.

## 论文阅读

## Multi-Agent Pathfinding with Continuous Time

### Abstract

- Most prior work on MAPF was on grids, assumed agents’ actions have uniform duration, and
  that time is discretized into timesteps.
- MAPF 之前的大部分工作都是在网格上进行的，假设代理的动作具有统一的持续时间，并且该时间被离散化为时间步长。 
- We propose a MAPF algorithm that does not rely on these assumptions, is complete, and provides provably optimal solutions. 提供了可证明的最优解
- This algorithm is based on a novel adaptation of Safe interval path planning (SIPP), a continuous time single-agent planning algorithm, and a modified version of Conflict-based search(CBS), a state of the art multi-agent pathfinding algorithm.

### 1 Introduction

- However, most prior work assumed that (1) time is discretized into time steps, (2) the duration of every action is one time step, and (3) in every time step each agent occupies exactly a single location.
- CCBS is based on a customized version of Safe interval path planning (SIPP) [Phillips and Likhachev, 2011], a continuous-time single-agent pathfinding algorithm, and an adaptation of Conflict-based search (CBS) [Sharon et al., 2015], a state-of-the-art multi-agent pathfinding algorithm.
- CCBS relies on the ability to accurately detect collisions between agents and to compute the safe intervals of each agent, that is, the minimal time an agent can start to move over an edge without colliding
- The results show that CCBS is feasible and outputs lower cost solutions compared to previously proposed algorithms; 结果表明，与先前提出的算法相比，CCBS 是可行的并且输出成本更低的解决方案 
- However, since CCBS considers agents’ geometry and continuous time, it can be slower than grid-based solutions, introducing a natural plan cost versus planning time tradeoff. 然而，由于 CCBS 考虑了代理的几何形状和连续时间，它可能比基于网格的解决方案慢，引入了自然计划成本与计划时间的权衡。
-  CCBS is the first MAPF algorithm that can handle non-unit actions duration, continuous time, non-grid domains, agents with a volume, and is still optimal and complete. CCBS 是第一个可以处理非单元动作持续时间、连续时间、非网格域、有体积的代理的 MAPF 算法，并且仍然是最优和完整的。 

### 2 Problem Definition

- it can handle agents moving with different speeds, using different roadmaps, and having complex shapes and sizes.
- A set of plans, one for each agent, is called a joint plan.
- To define cost-optimality of a MAPF R solution, we first define the cost of a plan π_i to be the sum of the durations of its constituent actions.
- The problem we address in this work is to find a solution to a given MAPF R problem that is optimal w.r.t its SOC, that is, no other solution has a lower SOC.

### 3 CBS with Continuous Times

- The typical CBS implementation considers two types of conflicts: a vertex conflict and an edge conflict.
- vertex conflict between plans π i and π j is defined by a tuple hi, j, v, ti and means that according to these plans agents i and j plan to occupy v at the same time t
- An edge conflict is defined similarly by a tuple hi, j, e, ti, and means that according to π i and π j both agents plan to traverse the edge e ∈ E at the same time, from opposite directions
- CBS runs two search algorithms: a low-level search algorithm that finds paths for individual agents subject to a given set of constraints, and a high-level search algorithm that chooses which constraints to add
- From CBS to CCBS
  - To detect conflicts, CCBS uses a geometry-aware collision detection mechanism. 为了检测冲突，CCBS 使用了几何感知碰撞检测机制 
    - Collision detection
    - conflicts can occur between agents traversing different edges, as well as vertex edge conflicts
  - To resolve conflicts, CCBS uses a geometry-aware unsafe-interval detection mechanism. 为了解决冲突，CCBS 使用了几何感知不安全间隔检测机制。
    - The high-level search in CCBS runs a best-first search like regular CBS, selecting in every iteration a leaf N of the CT that has the joint plan with the smallest cost
    - CCBS 中的高级搜索像常规 CBS 一样运行最佳优先搜索，在每次迭代中选择具有最小成本的联合计划的 CT 节点N。 
    - To compute the constraints to add to N_ and N_j , CCBS computes for each action its unsafe intervals w.r.t the other action
  - CCBS adds constraints over pairs of actions and time ranges, instead of location-time pairs. CCBS 添加了对动作和时间范围对的约束，而不是位置-时间对。 
  - For the low-level search, CCBS uses a version of SIPP adapted to handle CCBS constraints.
    - The low-level solver of CCBS is based on SIPP, which is a single-agent pathfinding algorithm designed to handle continuous time and moving obstacles
    - if a_i is a move action, add an action that represents waiting at v until t_ui and then moving to v`.
    - if a_i is a wait action, splitting it to two intervals [0, t i ] and [t ui , ∞).

- Theoretical Properties
  - CCBS is sound, complete, and optimal

### 4 Pratical Aspects

- There are various ways to detect collisions between agents with volume in a continuous space, including closed-loop geometric computations as well as sampling-based approaches. 检测agents之间碰撞的方法，闭环几何计算以及基于采样的方法 
- A general method for computing unsafe intervals is to apply the collision detection mechanism multiple time, starting from t i and incrementing by some small ∆ > 0 until the collision detection mechanism reports that the unsafe interval is done. 这种方法有一定的误差
- Conflict Detection and Selection Heuristics
  - 事实上，在我们的实验中，我们观察到冲突检测花费了大量时间。 
  - To speed up the conflict detection, we only checked conflicts between actions that overlap in time and may overlap geometrically. 为了加速冲突检测，我们只检查了时间重叠和几何重叠的动作之间的冲突。 
  - we implemented two heuristics for speeding up the detection process. 
    - past-conflicts heuristic. This implements the intuition that pairs of agents that have conflicted in the past are more likely to also conflict in the future.
    - To this end, we proposed a second hybrid heuristic approach
    - Initially, we detect all conflicts and choose only cardinal conflicts.  However, if a node N does not contain any cardinal or semi-cardinal conflict, then for all nodes in the CT subtree beneath it we switch to use the past-conflicts heuristic

### 5 Experimental Result

- Open Grids

  - The results show that increasing k yields solutions with lower SOC, as expected.

  - Increasing k, however, has also the effect of increasing the branching factor, which in turns means that path-finding becomes harder.

  - CBS was faster than CCBS, as its underlying solver is A ∗ on a 4-connected grid, detecting collisions is trivial, and it has only unit-time wait actions. However, even for k = 2, CCBS is able to find better　solutions, i.e., solutions of lower SOC.

- Dragon Age Maps

  - increasing k reduces the SOC and decreases the success rate.

- Conflict Detection and Resolution Heuristics
  - The results show that the proposed hybrid heuristic enjoys the complementary benefits of PastConf and Cardinals, expanding as few CT nodes as Cardinals and having the highest success rate. 结果表明，所提出的混合启发式方法享有 PastConf 和 Cardinals 的互补优势，扩展与 Cardinals 一样少的 CT 节点并具有最高的成功率

- Comparision to E-ICTS
  - The results show that for k = 2 and k = 3, CCBS works better in most cases, while E-ICTS outperforms CCBS for k = 4 and k = 5
  - Given an accurate unsafe interval detection mechanism, CCBS handles continuous time directly, and thus can return better solutions than E-ICTS

- We proposed CCBS, a sound, complete, and optimal MAPF algorithm that supports continuous time, actions with non-uniform duration, and agents and obstacles with a geometric shape.
- CCBS follows the CBS framework, using an adapted version of SIPP as a low-level solver, and unique types of conflicts and constraints in the high-level search
- Future work may apply meta-reasoning to decide when and how much to invest in conflict detection.

## Improving Continuous-time Conflict Based Search*

### PC (Prioritizing Conflicts)

- 如何选择要解决的冲突

  cardinal：主要冲突，两个子节点的cost高于父节点

  semi-cardinal：次要冲突，有一个子节点高于父节点

  non-cardinal：以上都不是

### DS (Disjoint Splitting)

- To address this inefficiency, CBS with DS (CBS-DS) introduces the notion of positive and negative constraints.
- 主要的思路，在子节点中增加 positive constraints. 也就是在一个节点中增加必须通过的点，也就是landmarks，这样可以减少 low-level搜索

### high-level heuristics

- 启发项的计算方式修改

- two admissible heuristics

  - H1 :  solving the following linear programming problem (LPP)
  - 最小化：$$\sum_{i=1}^nx_i $$
  - H2 : heuristic was based on identifying disjoint cardinal conflicts, which are cardinal conflicts between disjoint pairs of agents. we aim to choose the disjoint cardinal conflicts that would have the largest cost impact.
  - The H2 heuristic is the sum of the cost impacts of the chosen conflicts

  

## 代码阅读和总结

- 代码块：

  - config 读取模块
  - map处理模块
  - task处理模块
  - 算法模块：CBS
  - log 保存模块

- config读取模块

  - config.cpp config.h : 

  - const.h: 定义了相关的默认的参数，和XML file tags

  - 可以读取传入的参数，传入的方式是传入一个config.xml文件

    ```mermaid
    classDiagram
    	class Config{
    		Config(); //构造函数
    		getConfig(); // 获取需要的参数
    		
    	   double precious; //精度
    	   double focal_weight; // 权重
    	   bool use_cardinal; // 新的冲突检测的方法
    	   bool use_disjoint_splitting; //是否应用DS算法
    	   int hlh_type; // high level 类型
    	   double agent_size; //机器人大小参数
    	   double timelimit;   //查找到路径的时间限制，超过时间存在问题
    	}
    ```

    

- map 处理模块

  - 初始化时，传入了２个参数　Map map = Map(config.agent_size, config.connectdness); 

  -  map.get_map(argv[1]); // 通过传入的参数，完成地图的构造。传入的是一个map.xml文件

    ```mermaid
    classDiagram
    class Map {
    	-check_line()
    	-get_grid()
    	-get_roadmap()
    	+get_size()
    	+get_map()
    	+is_roadmap()
    	+cell_is_obstacle()
    	+get_width()
    	+get_gNode()
    	+get_id()
    	+get_i()
    	+get_j()
    	+get_vaild_moves()
    	+print_map()
    	+printPPM()
    	
    	-std::vector<std::vector<int>> grid
    	-std::vector<gNode> nodes
    	-std::vector<std::vector<Node>> valid_moves
    	-int  height
    	-int  width
    	-int  size
    	-int  connectedness
    	-double agent_size
    	-bool map_is_roadmap
    }
    ```

    - map中包含（loadmap结构）
      - nodes 主要的结构是g_node的容器，包含了所有的地图点的坐标，容器的索引代表地图路标点的标识，
      - gnode: 结构 坐标(i,j) 可连通的其他地图点的索引容器　vector<int> neighbors
      - vaid_move　每一个节点可以连通的下一个节点
      - map_is_roadmap  是否是roadmap的输入
      - 其他的是grid_map需要的参数

- task 处理模块

  - Task task; task.get_task(argv[2]);  // 通过传入发task.xml文件完成egents 初始位置和目标位置信息的输入

  - 信息模式

    ```xml
    <?xml version="1.0" ?>
    <root>
       <agent start_id="136" goal_id="50"/>
       <agent start_id="143" goal_id="169"/>
       <agent start_id="133" goal_id="165"/>
       <agent start_id="61" goal_id="96"/>
       <agent start_id="46" goal_id="123"/>
       <agent start_id="73" goal_id="108"/>
       <agent start_id="32" goal_id="67"/>
       <agent start_id="31" goal_id="66"/>
       <agent start_id="161" goal_id="32"/>
       <agent start_id="119" goal_id="154"/>
    </root>
    ```

  - ```mermaid
    classDiagram
          class Task{
          	  +get_task(); // 通过传入的信息，解析task
          	  +get_agent_size()
          	  +make_ids()
          	  +make_ij();  // satrt_id 和goal_id 转化到对应的坐标
          	  +get_agent()
          	  
              -std::vector<Agent> agents
          }
    ```




- 算法模块

  - CBS算法的主函数

    ```c++
    Solution find_solution(const Map &map, const Task &task, const Config &cfg);
    
    ```

    该函数通过已经解析好的"Map" "Task" "Config"的信息，调用CBS算法框架，返回最后得到的结果

  - 主函数中算法解析

    - 启发项初始化，初始化每一个egent到map中每一个节点的启发项的值，初始化为-1,然后为每一个agent 计算goal到地图中节点的欧氏距离启发项，数据保存在h_value容器中，为后面路径搜索提供启发项。

      ```c++
       h_values.init(map.get_size(), task.get_agents_size()); 
          for(int i = 0; i < int(task.get_agents_size()); i++)
          {
              Agent agent = task.get_agent(i);
              h_values.count(map, agent);
          }
      ```

    - 初始化CT, 如果初始化失败就结束，返回失败的结果

      ```C++
          if(!this->init_root(map, task))
              return solution;
      ```

      ```c++
      ├── init_root(const Map &map, const Task &task)
          // 循环遍历每一个agent
      	├── for(int i = 0; i < int(task.get_agents_size()); i++)
            	// 为每一个agent,利用SIPP算法获取optimal path,输入是agent,　地图，constraints, 启发项表
          	├── path = planner.find_path(agent, map, {}, h_values);
              // 把得到的路径保存在paths中
      		├── root.paths.push_back(path);
      	// 对比所有的egents的paths, 获得路径之间的冲突
      	 ├── auto conflicts = get_all_conflicts(root.paths, -1);
      	// 遍历所有的冲突，对冲突分类（PC 规则需要）
      	 ├── for(auto conflict: conflicts)
               // 计算将冲突解决之后得到新的paths
               ├──  auto pathA = planner.find_path(task.get_agent(conflict.agent1), map, {get_constraint(conflict.agent1, conflict.move1, conflict.move2)}, h_values);
      		 ├── auto pathB = planner.find_path(task.get_agent(conflict.agent2), map, {get_constraint(conflict.agent2, conflict.move2, conflict.move1)}, h_values);
               // 通过判断新找的两条路径的cost是否比以前有增加来划分冲突的类型
      		// 新找的两条路径都比以前大，归类为　cardinal_conflicts
      		├──  if(pathA.cost > root.paths[conflict.agent1].cost && pathB.cost > root.paths[conflict.agent2].cost)
                  ├──  root.cardinal_conflicts.push_back(conflict);
               // 新找的两条路径一条比以前大，归类为　semicard_conflicts
      		├──  else if(pathA.cost > root.paths[conflict.agent1].cost || pathB.cost > root.paths[conflict.agent2].cost)
                  ├── root.semicard_conflicts.push_back(conflict);
      		 // 新找的两条路径和以前一样，归类为　conflicts
      		├──  else
                  ├──  root.conflicts.push_back(conflict);
      	// 在tree中增加root节点
      	├──  tree.add_node(root);
      ```

    - 开始处理CT,　CBS的主要框架和算法

      ```c++
      // 循坏处理CT,直到不存在新的node产生
      ├──  do 
        	// 获取第一个cost最小的节点，同时会在CT中把当前的节点delete
          ├──  auto parent = tree.get_front();
      	├──  node = *parent;
      	// 获取所有的path
      	├──  auto paths = get_paths(&node, task.get_agents_size());
      	// 根据node中的冲突确认进行下一步	
      	// 如果没有冲突，就是解
      	├──   if(conflicts.empty() && semicard_conflicts.empty() && cardinal_conflicts.empty())
              ├──  break;
      	// 根据PC(prioritizing conflicts), 区分三种conficts, 一般的conflict, cardinal_conflicts, semicard_conflicts
      	// 优先处理　cardinal_conflicts，然后　semicard_conflicts，　最后是　一般的conflict
      	├──   if(!cardinal_conflicts.empty())
              	// 获取overcost 最大的conflict
              	├──  conflict = get_conflict(cardinal_conflicts);
      	├──  else if(!semicard_conflicts.empty())
              	├── conflict = get_conflict(semicard_conflicts);
      	├── else
              	├── conflict = get_conflict(conflicts);
      	// 根据conflict的两个aegnts, 分别构造constraints，并且增加新的约束
      	├── std::list<Constraint> constraintsA = get_constraints(&node, conflict.agent1);
      	├── Constraint constraintA(get_constraint(conflict.agent1, conflict.move1, conflict.move2));
          ├──  constraintsA.push_back(constraintA);
      	// 由新的约束，查找新的路径
          ├──  pathA = planner.find_path(task.get_agent(conflict.agent1), map, constraintsA, h_values);
          ├──  pathB = planner.find_path(task.get_agent(conflict.agent2), map, constraintsB, h_values);
         // 由新的路径构造俩个子节点
      	├──  CBS_Node right({pathA}, parent, constraintA, node.cost + pathA.cost - get_cost(node, conflict.agent1), 0, node.total_cons + 1);
          ├──  CBS_Node left({pathB}, parent, constraintB, node.cost + pathB.cost - get_cost(node, conflict.agent2), 0, node.total_cons + 1);
      	// DS 算　在constraintA或则constraintB中增加 positive_constraint
      	// 分别计算constraintA和constraintB中positive_constraint的数量
      	 ├──  for(auto c: constraintsA)
                      ├── if(c.positive)
                         ├──  agent1positives++;
           ├──  for(auto c: constraintsB)
                     ├──  if(c.positive)
                          ├── agent2positives++;
      	// 通过positive_constraint的数量决定在那个约束容器里添加新的positive_constraint
      	// 如果不是等待的约束并且在agent2中的positive_constraint更多，那么在constraintsB中增加positive_constraint
      	├── if(conflict.move1.id1 != conflict.move1.id2 && agent2positives > agent1positives && pathA.cost > 0)
              //构造positive_constraint的约束
                ├──positive = Constraint(conflict.agent1, constraintA.t1, constraintA.t2, conflict.move1.id1, conflict.move1.id2, true);
      		// 查看是否已经存在了当前的positive约束
                ├── if(check_positive_constraints(constraintsA, positive))
                    // 将positive 约束加入到新的node中
                	 ├── left.positive_constraint = positive;
      		  	 ├── constraintsB.push_back(left.positive_constraint);
      		// 如果上面的判断失败，没有在constraintsB插入constraintsB，考虑在constraintA中插入
      		├── if(conflict.move2.id1 != conflict.move2.id2 && !inserted && pathB.cost > 0)
               	├──  positive = Constraint(conflict.agent2, constraintB.t1, constraintB.t2, conflict.move2.id1, conflict.move2.id2, true);
      			├── if(check_positive_constraints(constraintsB, positive))
                       ├── constraintsA.push_back(right.positive_constraint);
      		// 如果以上的条件都没有满足，pathB.cost < 0, 那么在constraintsB中增加positive_constraint
      		├──  if(conflict.move1.id1 != conflict.move1.id2 && !inserted && pathA.cost > 0)
                  ├──  constraintsB.push_back(left.positive_constraint);
      
      		//  如果pathA存在，且检查不存在相同的constraint, 其中一个是positive的约束，一个是其他的约束。
      		├── if(right_ok && pathA.cost > 0 && validate_constraints(constraintsA, pathA.agentID)
                  //  更新新的node的冲突的消息
                 ├── find_new_conflicts(map, task, right, paths, pathA, conflicts, semicard_conflicts, cardinal_conflicts, low_level_searches, low_level_expanded);
                  // 新的node中可以找到路径
                 ├──  if(right.cost > 0)
                      // 通过cardinal_conflicts求解启发项, 启发项增强点
                    ├──  right.h = get_hl_heuristic(right.cardinal_conflicts); 
                      // cost加上启发项
                    ├── right.cost += right.h; 
                       // 添加node到CT上
                    ├── tree.add_node(right); 
              // pathB相同的方法
       		// 如果消耗的时间查过限制，返回fasle
               ├── if(time_spent.count() > config.timelimit)
                   ├──  solution.found = false;
                     ├── break;  
      ├── while(tree.get_open_size() > 0);

  - 启发项计算
    
    ```c++
    double CBS::get_hl_heuristic(const std::list<Conflict> &conflicts)
        
    ```
    
  - 启发项表的计算
  
    heuristic.cpp
  
  
  
  - 启发项
  
  - 冲突检查方法
  
    检查冲突的函数，输入是 两个agents的paths
  
    ```c++
    Conflict CBS::check_paths(const sPath &pathA, const sPath &pathB)
        
  
  
  
  - 结构：
  
    ```c++
    struct Conflict
    {
        int agent1, agent2;
        double t;
        Move move1, move2;  mov1 是agent1, move2指的是agent2
        double overcost;    //新的路径比之前的路径增加的cost
        //sPath path1, path2;
        Conflict(int _agent1 = -1, int _agent2 = -1, Move _move1 = Move(), Move _move2 = Move(), double _t = CN_INFINITY)
            : agent1(_agent1), agent2(_agent2), t(_t), move1(_move1), move2(_move2) {overcost = 0;}
        bool operator < (const Conflict& other)
        {
            return this->overcost < other.overcost;
        }
    };
    
    ```
  
    - node 结构分析
  
      ```c++
      struct Node
      {
          int     id;
          double  f, g, i, j;
          Node*   parent;
          std::pair<double, double> interval;
          int interval_id;
          Node(int _id = -1, double _f = -1, double _g = -1, double _i = -1, double _j = -1, Node* _parent = nullptr, double begin = -1, double end = -1)
              :id(_id), f(_f), g(_g), i(_i), j(_j), parent(_parent), interval(std::make_pair(begin, end)) {interval_id = 0;}
          bool operator <(const Node& other) const //required for heuristic calculation
          {
              return this->g < other.g;
          }
      };
      ```
  
    
    - 代码中相关参数的说明
    
      ```c++
      //参数：　
      bool use_disjoint_splitting;　// 是否应用DS算法，主要的功能: efficiency in the high-level search
      
      bool use_cardinal;　// 冲突检测的新方法，增加两个 heuristics 加速冲突的检测
       
      int hlh_type;       // 和 node.h参数相关　get_hl_heuristic　启发项的应用
      
      double CBS::get_hl_heuristic(const std::list<Conflict> &conflicts)
      ```
    
    - path结构
    
      ```c++
      struct sPath
      {
          std::vector<sNode> nodes;
          double cost;   // 欧式距离
          int agentID;
          int expanded;
          sPath(std::vector<sNode> _nodes = std::vector<sNode>(0), double _cost = -1, int _agentID = -1)
              : nodes(_nodes), cost(_cost), agentID(_agentID) {expanded = 0;}
          sPath operator= (const Path &path)
          {
              cost = path.cost;
              agentID = path.agentID;
              expanded = path.expanded;
              nodes.clear();
              for(auto n:path.nodes)
                  nodes.push_back(sNode(n));
              return *this;
          }
      };
      ```
    
      
    
  - SIPP
  
    主函数
    
    ```c++
    Path SIPP::find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values）
    ```
    
    输入是：单个agent，地图信息：map；约束的消息：cons；启发项信息：h_values
    
    各个参数的结构：
    
    ```c++
    // agent构造
    struct Agent
    {
        // agent 的起始点和目标点的位置，i j分别表示坐标
        double start_i, start_j, goal_i, goal_j;
        //  起始点和目标点对应的id
        int start_id, goal_id;
        //agent 的id
        int id;
        //  agent 几何大小，半径
        double size;
        Agent(int s_id = -1, int g_id = -1, int _id = -1)
            :start_id(s_id), goal_id(g_id), id(_id) {}
    };
    
    // constraints 结构
    struct Constraint
    {
        // 约束所属的agent
        int agent;
        double t1, t2; // prohibited to start moving from (i1, j1) to (i2, j2) during interval (t1, t2)
        //double i1, j1, i2, j2; // in case of node constraint i1==i2, j1==j2.
        // move中node的id
        int id1, id2;
        // 约束是否是postive
        bool positive;
        Constraint(int _agent = -1, double _t1 = -1, double _t2 = -1, int _id1 = -1, int _id2 = -1, bool _positive = false)
            : agent(_agent), t1(_t1), t2(_t2), id1(_id1), id2(_id2), positive(_positive) {}
        friend std::ostream& operator <<(std::ostream& os, const Constraint& con)
        {
            os<<con.agent<<" "<<con.t1<<" "<<con.t2<<" \n";//<<con.i1<<" "<<con.j1<<" "<<con.i2<<" "<<con.j2<<"\n";
            return os;
        }
        bool operator <(const Constraint& other) const
        {
            return t1 < other.t1;
        }
    };
    
    // 启发项表
    class Heuristic
    {
        //  启发项表，每一个节点到goal之间的欧式距离
        std::vector<std::vector<double>> h_values;
        //  多索引容器
        Open_Container open;
        //  构造启发项表时计算最小的node
        Node find_min();
        // 计算欧式距离
        double dist(const Node& a, const Node& b){ return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2)); }
    public:
        Heuristic(){}
        void init(unsigned int size, unsigned int agents);
        // 通过地图信息，计算启发项
        void count(const Map &map, Agent agent);
        unsigned int get_size() const {return h_values[0].size();}
        // 通过启发项表，获取对应的启发项的值
        double get_value(int id_node, int id_agent) { return h_values[id_node][id_agent]; }
    };
    
    ```
    
    
    
  - Path SIPP::find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values
  
    



