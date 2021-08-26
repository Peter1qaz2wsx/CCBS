# SIPP: Safe Interval Path Planning for Dynamic Environments

# 动态环境的安全间隔路径规划 

- dynamic environments  ----- time as additional dimension

- most real-time approaches to path planning treat dynamic obstacles as static and constantly re-plan as dynamic obstacles move. 动态物体移动之后，重新规划路径, 尽管获得了效率，但这些方法牺牲了最优性甚至完整性。
- 安全间隔：在一定的“configuration”时间下没有冲突
- 规划器利用观测，构建搜索空间。搜索空间的状态量通过“configuration”和安全间隔定义，这样可以得到一个图，图中一个配置“configuration”只有几个状态量。
- 实验结果表明：在很多动态物体的情况下，和其他的算法对比明显更快

## 1. INTRODUCTION

- When planning in dynamic environments, adding a time dimension to the state-space is needed in order to properly handle moving obstacles
- In this paper we propose a method that exploits the observation that the number of contiguous safe intervals is generally significantly smaller than the number of timesteps that compose those intervals.
- A configuration is the set of non-time variables that describe the robot’s state, such as position, heading, joint angles, etc 配置是描述机器人状态的一组非时间变量，例如位置、航向、关节角度等 
- configuration: 可以认为是机器人的状态，但是不包含时间信息

## 2 RELATED WORK

- HCA*

## 3. ALGORITHM

- We define a safe interval as a contiguous period of time for a configuration, during which there is no collision and it is in collision one timestep prior and one timestep after the period.

- A collision interval is a contiguous period of time for a configuration, where each timestep in this period is in collision with a dynamic obstacle and it is safe one timestep prior and one timestep after the period.  碰撞间隔是状态置的连续时间段，其中该时间段中的每个时间步都与动态障碍物发生碰撞，并且在该时间段之前一个时间步和之后一个时间步是安全的 

- Each spatial configuration (such as the one shown in Figure 2) has a timeline (Figure 3), which is just an ordered list of intervals, alternating between safe and collision. 每个空间状态（如图 2 所示）都有一个时间轴（图 3），它只是一个有序的间隔列表，在安全和碰撞之间交替。

  ![](/home/liukang/文档/20210802/imags/选区_019.png)

- Using safe intervals to represent the time dimension of the search space is what makes our algorithm efficient, and is the key idea to our approach. 使用安全区间来表示搜索空间的时间维度是使我们的算法高效的原因，也是我们方法的关键思想。 

- Dynamic Obstacle Representation
  - We are given a list of dynamic obstacles, where each obstacle has a radius and a trajectory

- Notations and Assumptions
  - g(s): cost form start to s
  - h(s) cost form s to goal status
  - c(s, s1 ) = time to execute the action from s to s1 . In other words, the goal of the planner is to find a time- minimal trajectory.
  - The robot is capable of waiting in place (this assumption would not be true of a motorcycle).
  - Inertial constraints (acceleration/deceleration) are negligible. The planner assumes the robot can stop and accelerate instantaneously.

- Planning with Safe Intervals

  - Graph Construction

    - 当规划器被初始化时，我们使用预测的动态障碍物轨迹为每个空间configuration创建一个时间线。 这是通过沿着每个动态障碍物的轨迹迭代每个点并更新该点碰撞距离内所有配置的时间线来完成的

  - Graph Search

    - In Figure 5, the function M (s) returns the motions that can be performed from state s.
    - 

    ![](/home/liukang/文档/20210701/CBS/选区_005.png)

  ![](/home/liukang/文档/20210701/CBS/选区_006.png)

  - start_t   到达下一个节点的最早时间，end_t　到达下一个节点的最晚时间

  - startTime(i) 安全间隔i的初始时间，endTime(i)安全间隔i的结束时间
  - updateTime(S‘)：we also replace the time value stored in state s’ with this new shorter time
  - ![](/home/liukang/文档/20210701/CBS/选区_009.png)
  - ![](/home/liukang/文档/20210701/CBS/选区_008.png)
  - successors(S_A0, S_C0, S_C1)

- Theoretical Analysis
  - Theorem 1: Arriving at a state at the earliest possible time guarantees the maximum set of possible successors.
  - Theorem 2: When the safe interval planner expands a state in the goal configuration, it has found a time-minimal, collision-free path to the goal.
  - Theorem 3: If the configuration with the most dynamic obstacles passing through it has n such occurrences, then each configuration can have at most n + 1 safe intervals.

## 4 EXAMPLE

- ![](/home/liukang/文档/20210701/CBS/选区_010.png)

- start R -> gobal G 

## 5 EXPERIMENTAL RESULTS

- ![](/home/liukang/文档/20210701/CBS/选区_011.png)

- ![](/home/liukang/图片/选区_012.png)

## 代码理解

主函数

```c++
Path SIPP::find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values）
```

输入是：单个agent，地图信息：map；约束的消息：cons；启发项信息：h_values

```c++
// 主函数
├──  find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values）
     // 将约束信息加入到算法中。如果positive = true, 作为landmarks使用，并且按照时间排序
     //positive = false, 如果是wait，那么作为冲突间隔，是move添加到 move_constraint中 
    ├──  make_constraints(cons);
    // 如果约束中存在landmark约束
    ├── if(!landmarks.empty())
    	// 遍历landmark约束
          ├── for(unsigned int i = 0; i <= landmarks.size(); i++)
               // 如果是第一个landmark 约束. 获取地图起始和需要到达的节点
               ├──  if(i == 0)
               		// 起始的位置
               		├── starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
               　// 第一个landmark的初始位置，也是第一个goal位置
               		├── goals = get_endpoints(landmarks[i].id1, map.get_i(landmarks[i].id1), map.get_j(landmarks[i].id1), landmarks[i].t1, landmarks[i].t2);
              // 不是第一个约束，重新赋值starts和goals的值
               ├── else
               		 ├──  starts.clear();
               		 ├──  for(auto p:results)
                     ├──   starts.push_back(p.nodes.back());
               		 ├──  if(i == landmarks.size())
                    	├── goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY).back()};
                	├── else
                    ├──  goals = get_endpoints(landmarks[i].id1, map.get_i(landmarks[i].id1), map.get_j(landmarks[i].id1), landmarks[i].t1, landmarks[i].t2);
               // 如果没有goals，返回空
              ├── if(goals.empty())
             	 	├──return Path();
               // 根据开始和结束的目标点，搜索部分的路径。应用的是Ａ* 算法
                ├── parts = find_partial_path(starts, goals, map, h_values, goals.back().interval.second);
               
          
```



- 其他函数分析

  ```C++
  // 构造算法中需要的约束,将传入的约束进行分类处理。
  // positive = true, 作为landmarks使用，并且按照时间排序
  // positive = false, 如果是wait，那么作为冲突间隔，是move添加到 move_constraint中
  void SIPP::make_constraints(std::list<Constraint> &cons)
  {
      for(auto con : cons)
      {
          if(con.positive == false)
          {
              if(con.id1 == con.id2) // wait consatraint
                  //将冲突间隔加入到  collision_intervals容器中
                  add_collision_interval(con.id1, std::make_pair(con.t1, con.t2));
              else
                  // move constraints
                  //  是move添加到 move_constraint中 constraints容器
                  add_move_constraint(Move(con));
          }
          // 如果是positive约束，也就是必须经过的约束。加入到landmarks中
          else
          {
              bool inserted = false;
              for(unsigned int i = 0; i < landmarks.size(); i++)
                  if(landmarks[i].t1 > con.t1)
                  {
                      landmarks.insert(landmarks.begin() + i, Move(con.t1, con.t2, con.id1, con.id2));
                      inserted = true;
                      break;
                  }
              if(!inserted)
                  landmarks.push_back(Move(con.t1, con.t2, con.id1, con.id2));
          }
      }
  }
  
  // constraints类型分析
  //  std::map<std::pair<int, int>, std::vector<Move>> constraints;//stores sets of constraints associated with moves
  
  // node结构，这里的node是sipp算法中产生的node
  struct Node
  {
      int     id;　　//指的是地图站点的id
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
  
  

- landmark的使用方法：

- ```c++
  //应用方式，获取一个node的容器
  get_endpoints(landmarks[i].id1, map.get_i(landmarks[i].id1), map.get_j(landmarks[i].id1), landmarks[i].t1, landmarks[i].t2)
  // 输入第一个站点的id，站点对应的坐标，landmark的起始时间和结束时间。没有landmark_id2的信息
      std::vector<Node> SIPP::get_endpoints(int node_id, double node_i, double node_j, double t1, double t2)
  {
      std::vector<Node> nodes;
      //　用输入的信息初始化nodes
      nodes = {Node(node_id, 0, 0, node_i, node_j, nullptr, t1, t2)};
      // collision_intervals 中该路标站点不存在wait的约束
      if(collision_intervals[node_id].empty())
          return nodes;
      // 存在wait 的约束，进行调整
      //将约束的信息加入到node中？
      else
          for(unsigned int k = 0; k < collision_intervals[node_id].size(); k++)
          {    
              unsigned int i(0);
              while(i < nodes.size())
              {
                  Node n = nodes[i];
                  //　ｃ表示某一个等待的约束
                  auto c = collision_intervals[node_id][k];
                  bool changed = false;
                  //　如果移动时间在不可wait的时间间隔内，存在问题
                  if(c.first - CN_EPSILON < n.interval.first && c.second + CN_EPSILON > n.interval.second)
                  {
                      nodes.erase(nodes.begin() + i);
                      changed = true;
                  }
                  //　移动的下限大于不可"wait"的下限，同时小于上限，但是移动的上限大于“wait”的上限
                  else if(c.first - CN_EPSILON < n.interval.first && c.second > n.interval.first)
                  {
                      // node的间隔时间初始时间为不可等待的上限时间
                      nodes[i].interval.first = c.second;
                      changed = true;
                  }
                  // 移动的下限小于不可"wait"的下限，移动的上限大于不可"wait"的上限，
                  else if(c.first - CN_EPSILON > n.interval.first && c.second + CN_EPSILON < n.interval.second)
                  {
                      // 可通行时间的上限是 wait的下限，同时增加一个可通行时间间隔
                      nodes[i].interval.second = c.first;
                      nodes.insert(nodes.begin() + i + 1, Node(node_id, 0, 0, node_i, node_j, nullptr, c.second, n.interval.second));
                      changed = true;
                  }
                  // 移动的上限大于不可"wait"的下限，移动的上限小于不可"wait"的上限，
                  else if(c.first < n.interval.second && c.second + CN_EPSILON > n.interval.second)
                  {
                      // 可通行时间的上限是 wait的下限
                      nodes[i].interval.second = c.first;
                      changed = true;
                  }
                  if(changed)
                  {
                      i = -1;
                      k = 0;
                  }
                  i++;
              }
          }
      return nodes;　//　返回的是nodes包含的信息：每一个元素是包含不同的可通行时间间隔的node的信息
  }
   
      
  ```

- sipp　搜索的算法

- ```c++
  // 这里实际是sipp搜索的算法，输入是起始的node和可通行的时间间隔，目标node和可通行的时间间隔。每一个node包含一段可通行的时间间隔，h_value启发项表，可以直接获取当前点到目标点的启发项，max_f 最大的可通行的时间。主体的算法框架是基于A*算法
  std::vector<Path> SIPP::find_partial_path(std::vector<Node> starts, std::vector<Node> goals, const Map &map, Heuristic &h_values, double max_f)
  {
      open.clear();
      close.clear();
      path.cost = -1;
      visited.clear();
      std::vector<Path> paths(goals.size());
      int pathFound(0);
      for(auto s:starts)
      {
          s.parent = nullptr;
          open.push_back(s);
          //  std::unordered_map<int, std::pair<double, bool>> visited;
          //  s.id 地图中站点的id, s.interval_id * map.get_size() 乘以map中站点的数量作为键值
          // s.interval_id 第几个可通行时间段？visited存储节点
          visited.insert({s.id + s.interval_id * map.get_size(), {s.g, false}});
      }
      Node curNode;
      while(!open.empty())
      {
          // 找到第一个初始的节点，并且把它从容器中删除
          curNode = find_min();
          // 该节点是否在visited容器中
          auto v = visited.find(curNode.id + curNode.interval_id * map.get_size());
          if(v->second.second)
              continue;
          // 该节点访问过
          v->second.second = true;
          // std::unordered_map<int, Node> close;
          auto parent = &close.insert({curNode.id + curNode.interval_id * map.get_size(), curNode}).first->second;
        // 如果当前的点是目标点
          if(curNode.id == goals[0].id)
          {
              for(unsigned int i = 0; i < goals.size(); i++)
                  if(curNode.g - CN_EPSILON < goals[i].interval.second && goals[i].interval.first - CN_EPSILON < curNode.interval.second)
                  {
                      paths[i].nodes = reconstruct_path(curNode);
                      if(paths[i].nodes.back().g < goals[i].interval.first)
                      {
                          curNode.g = goals[i].interval.first;
                          paths[i].nodes.push_back(curNode);
                      }
                      paths[i].cost = curNode.g;
                      paths[i].expanded = int(close.size());
                      pathFound++;
                  }
              if(pathFound == int(goals.size()))
                  return paths;
          }
          std::list<Node> succs;
          succs.clear();
          find_successors(curNode, map, succs, h_values, Node(goals[0].id, 0, 0, goals[0].i, goals[0].j));
          std::list<Node>::iterator it = succs.begin();
          while(it != succs.end())
          {
              if(it->f > max_f)
              {
                  it++;
                  continue;
              }
              it->parent = parent;
              add_open(*it);
              it++;
          }
      }
      return paths;
  }
  
  ```
  
- find_successors 函数

- ```c++
  void SIPP::find_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values, Node goal)
  {
      Node newNode;
      // 获取相邻的可以同行的node
      std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
      for(auto move : valid_moves)
      {
          newNode.i = move.i;
          newNode.j = move.j;
          newNode.id = move.id;
          double cost = dist(curNode, newNode);
          newNode.g = curNode.g + cost;
          std::vector<std::pair<double, double>> intervals(0);
          auto colls_it = collision_intervals.find(newNode.id);
          if(colls_it != collision_intervals.end())
          {
              std::pair<double, double> interval = {0, CN_INFINITY};
              for(unsigned int i = 0; i < colls_it->second.size(); i++)
              {
                  interval.second = colls_it->second[i].first;
                  intervals.push_back(interval);
                  interval.first = colls_it->second[i].second;
              }
              interval.second = CN_INFINITY;
              intervals.push_back(interval);
          }
          else
              intervals.push_back({0, CN_INFINITY});
          auto cons_it = constraints.find({curNode.id, newNode.id});
          int id(0);
          for(auto interval: intervals)
          {
              newNode.interval_id = id;
              id++;
              auto it = visited.find(newNode.id + newNode.interval_id * map.get_size());
              if(it != visited.end())
                  if(it->second.second)
                      continue;
              if(interval.second < newNode.g)
                  continue;
              if(interval.first > newNode.g)
                  newNode.g = interval.first;
              if(cons_it != constraints.end())
                  for(unsigned int i = 0; i < cons_it->second.size(); i++)
                      if(newNode.g - cost + CN_EPSILON > cons_it->second[i].t1 && newNode.g - cost < cons_it->second[i].t2)
                          newNode.g = cons_it->second[i].t2 + cost;
              newNode.interval = interval;
              if(newNode.g - cost > curNode.interval.second || newNode.g > newNode.interval.second)
                  continue;
              if(it != visited.end())
              {
                  if(it->second.first - CN_EPSILON < newNode.g)
                      continue;
                  else
                      it->second.first = newNode.g;
              }
              else
                  visited.insert({newNode.id + newNode.interval_id * map.get_size(), {newNode.g, false}});
              if(goal.id == agent.goal_id) //perfect heuristic is known
                  newNode.f = newNode.g + h_values.get_value(newNode.id, agent.id);
              else
              {
                  double h = sqrt(pow(goal.i - newNode.i, 2) + pow(goal.j - newNode.j, 2));
                  for(unsigned int i = 0; i < h_values.get_size(); i++) //differential heuristic with pivots placed to agents goals
                      h = std::max(h, fabs(h_values.get_value(newNode.id, i) - h_values.get_value(goal.id, i)));
                  newNode.f = newNode.g + h;
              }
              succs.push_back(newNode);
          }
      }
  }
  
  ```

- 后面继续

  - 现在的进展add_open函数find_partial_path函数

- open容器：保存从初始node开始，循环保存下一步的可达的node。同时是按照f值排序的容器。按顺序访问容器中的node，通过visited判断之前是否已经访问过，访问过说明之前有一个更小的f值，可以直接跳过。
- visited 容器：在find_successors函数中向容器中插入元素。
  -  if(it->second.second) 表示的含义：当前的这个node已经作为初始node遍历下一步了，这表明现在没有必要在进行一次了，同时之前的也是f值更小的node
  - 主要的功能：计算从start开始到某个节点时最小的g。同时会计算h，更新f的值
- close容器记录在open容器中pop出来的元素并且不是重复搜索的节点

## 判断是wait的代码

- recostruct_path函数
- 如果是move把g*scalar(4) 增加move的g的权重



## 增加move的cost权重，使得优先得到wait的路径

1. 目前算法实现的方式可能在那个地方可以修改。

2. 遍历path，如果存在相同的节点。判断在相同节点之间的时间间隔是否在该节点的安全间隔内，如果在安全时间内，将相同的节点之间的时间改成wait。

   1. 需要确认的点：节点的安全时间有没有修改？

      ```c++
              std::vector<std::pair<double, double>> intervals(0);
              auto colls_it = collision_intervals.find(newNode.id);
              if(colls_it != collision_intervals.end())
              {
                  //　把该节点的安全时间间隔保存在intervals中
                  std::pair<double, double> interval = {0, CN_INFINITY};
                  for(unsigned int i = 0; i < colls_it->second.size(); i++)
                  {
                      interval.second = colls_it->second[i].first;
                      intervals.push_back(interval);
                      interval.first = colls_it->second[i].second;
                  }
                  interval.second = CN_INFINITY;
                  intervals.push_back(interval);
              }
              else
                  intervals.push_back({0, CN_INFINITY});
      // 根据这段代码修改
      
      
        if(fabs(path.nodes[j].g - path.nodes[i].g - dist(path.nodes[j], path.nodes[i])) > CN_EPSILON)
              {
                  Node add = path.nodes[i];
                  add.g = path.nodes[j].g - dist(path.nodes[j], path.nodes[i]);
                  // 增加一个新的node，和前面的node形成等待的行为
                  path.nodes.emplace(path.nodes.begin() + j, add);
              }
      
      Path SIPP::check_same_node(Path path) {
          bool flag = true;
          while(flag) {
              std::vector<Node> nodes;
        	  	nodes = path.nodes;
              flag = false;
              for (unsigned int i = 0; i < nodes.size(); ++i) {
                  int id1 = nodes[i].id;
                  for (unsigned int j = i+1; j<nodes.size(); ++j) {
                      int id2 = nodes[j].id;
                      if (id1 == id2)
                          if(can_fusion_node(id1, id2, path)) {
                              path = fusion_path(path, std::pair<int>(id1, id2));
                              flag = true;
                              break;
                          }
                  }
               	if (falg)
                     break;
              }
          }
          return path;
      }
      
      bool can_fusion_node(int id1, int id2, Path path) {
          std::vector<Node> nodes;
          nodes = path.nodes;
          int id = nodes[id1].id;
          double g1 = nodes[id1].g;
          double g2 = nodes[id2].g;
          auto colls_it = collision_intervals.find(id);
           std::vector<std::pair<double, double>> intervals(0);
          if(colls_it != collision_intervals.end()) {
              std::pair<double, double> interval = {0, CN_INFINITY};
              for(unsigned int j = 0; j < colls_it->second.size(); j++) {
                  interval.second = colls_it->second[i].first;
                  intervals.push_back(interval);
                  interval.first = colls_it->second[i].second;
              }
          }
          for (auto interval:intervals) {
              if ((interval.first > g1) && (interval.second>g2))
                  return true;
          }
          return false;
      }
      ```
      
      
