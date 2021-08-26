# SIPP: Safe Interval Path Planning for Dynamic Environments

# 动态环境的安全间隔路径规划 

- dynamic environments  ----- time as additional dimension

- most real-time approaches to path planning treat dynamic obstacles as static and constantly re-plan as dynamic obstacles move. 动态物体移动之后，重新规划路径, 尽管获得了效率，但这些方法牺牲了最优性甚至完整性。
- 安全间隔：在一定的“configuration”时间下没有冲突
- 规划器利用观测，构建搜索空间。搜索空间的状态量通过“configuration”和安全间隔定义，这样可以得到一个图，图中一个配置“configuration”只有几个状态量。
- 实验结果表明：在很多动态物体的情况下，明显更快和其他的算法对比

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
               		├── starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
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

