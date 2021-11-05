# Conflict-based search for optimal multi-agent pathfinding

 - In many cases this two-level formulation enables CBS to examine fewer states than A*
 while still maintaining optimality
 - MA-CBS is a generalization of CBS. MA-CBS allows agents to be merged into small groups of joint agents
 - MA-CBS is a framework that can be built on top of any optimal and complete MAPF solver
 in order to enhance its performance.
 - 介绍了MA-CBA 框架，可以用在其他的MAPF求解方法上。本文用在了CBS上

## Introduction

 - 单机planning一般都是用A*

 - MAPF: consists of a graph and a number of agent. a unique start state and a unique goal state are given
 - task is to find paths for all agents from their start states to their goal states
 - Finding an optimal solution for the MAPF problem is NP-hard [56], as the state space grows exponentially with the number of agents. (NP-hard)
 - Therefore, the traditional approach for solving MAPF optimally is
 by using A*-based searches [35,45]  (A*-based)
 - search algorithms that are based on A* can solve this problem optimally, but they may run for a very long time or exhaust the available memory.
 - The first part of the paper gives a survey on MAPF research.
 - In the second part of the paper we introduce a new approach for optimally solving MAPF
 - We study the behavior of our CBS algorithm and discuss its advantages and drawbacks when compared to A*-based approaches as well as other approaches
 - While CBS is ineffective in some cases, there are many cases where CBS outperforms EPEA* [15,20]  (EPEA*) --> the state-0f-the-art A*-based approach
 - Results show the superiority of CBS over the A*-based approaches　and ICTS [42], another recent approach, on many of these domains.  (TCTS)
 - we mitigate the worst-case performance of CBS by generalizing CBS into a new algorithm called Meta-agent CBS (MA-CBS).
 - This paper contains a more comprehensive description of the CBS algorithm and the MA-CBS framework, with broader theoretical analysis and experimental comparisons　to other MAPF algorithms.

## Problem definition and terminology
  - Ｐroblem input: graph G(V,E)
  - k agents labeled a1,a2 . . . ak. Every agent ai has a start vertex, start_i ∈ V and a goal vertex, goal_i ∈ V.
  - Actions: Between successive time points, each agent can perform a move action to a neighboring vertex or a wait action to stay idle at its current vertex.
  - MAPF constraints: vetex at most one agent at a given time, one agent to traverse the same edge
  - MAPF task: a set of non-conflicting paths, one for each agent, where a path for agent a i is a
    sequence of { move , wait } actions, from start_i to goal_i
  - Cost function: 误差函数
   - minimizing a global cumulative cost function
   - We describe the algorithms in this paper in the context of a common cost function that we call the sum-of-costs [12,45,42,38,39].
   - the minimal sum-of-costs, has been shown to be NP-hard [56]
   - Makespan：minimizes the total time until the last agent reaches its destination
   - corresponding to the total amount of distance traveled by all agents;The Fuel cost function is in a fact the sum-of-costs of all agents where only move actions incur costs but wait actions are free
   - giving different weights to different agents is also possible
   - else
  - Distributed vs. centralized
   - MAPF problems can be categorized into two groups: distributed and centralized.
   - In a distributed setting, each agent has its own computing power and different communication paradigms may be assumed
   - the centralized setting assumes a single central computing power which needs to find a solution for all agents.

## Survey of centralized MAPF algorithms
 - Work assuming a centralized approach can be divided into three classes

 - Reduction-based solvers
  - mProminent examples include reducing to Boolean Satisfiability (SAT) [50], Integer Linear Programming (ILP) [55] and Answer Set Programming (ASP) [13].
  - These methods return the optimal solution and are usually designed for the makespan cost function.
  - On large problem instances the translation process from an MAPF instance to the required problem has a very large, yet polynomial, overhead which makes these approaches inefficient.
  - 问题转化的过程中，当问题是大型的问题时，有非常大的多项式开销

 - MAPF-specific sub-optimal solvers
  - Algorithms of this class are usually highly efficient but do not guarantee optimality and even completeness in some cases.
  - number of agents is large
  - Search-based suboptimal solvers
   - Hierarchical Cooperative A* (HCA start) [43]
      	- When searching for a path for a later agent, paths chosen by previous agents are blocked.
   - Windowed-HCA* (WHCA start) [43]
        - only performs cooperative pathfinding within a limited window, after which other agents are ignored
   - 仅在有限的窗口内执行协作寻路，之外的将被忽略。
   - A perfect single-agent heuristic is most often used to guide this search. limited memory, heuristic calculated at runtime
   - [48] reduce heuristics
   - WHCA* was enhanced [4]
   - drawbacks: too many agents exist, deadlocks may occur，HCA* is not guaranteed to be complete；
      HCA* does not　provide any guarantees on the quality of its solution；Finally,HCA* may even slow the search significantly. 
   - 由于各个代理独立地寻找最小长度的解决方案，因此当有大量可用空间时，代理可能会发生不必要的冲突，从而产生大量的计算成本来解决
 - Rule-based suboptimal solvers
  - 基于规则的求解器倾向于以低计算成本实现完整性而不是解决方案质量 
  - TASS [25] and Push and Swap (and its variants) [30,37,10] are two recently proposed rule-based MAPF sub-optimal algorithms that run in polynomial time. 
  - Both algorithms use a set of “macro” operators.
  - Both TASS and Push and Swap do not return an optimal solution and guarantee completeness for special cases only.
  - TASS： only for tree graph, at least two vertices are always unoccupied
 - Hybrid solvers
  - 这些方法对于一般情况并不完整，因为瓶颈可能会发生死锁
  - Another hybrid solver was presented by Wang and Botea [53], it is only proven to be complete for grids which have the slidable property
  - uses abstraction to reduce the size of the state-space [35] CSP. Each special subgraph adds constraints to the CSP solver, making the CSP solver faster [36]
  - Finding the optimal partitioning is a hard problem and not always feasible.Open spaces are not suitable for partitioning by the defined structures making this algorithm less effective on graphs with open spaces.
  - 这里介绍了一些混合的算法。主要包含三种：1,特定的运动规则以及重要的搜索 2, 预计算所有的路径，主要针对的是grids; 3, 基于搜索的算法，通过抽象减少状态空间的尺寸。将graph分解成多个subgraph,子图中包含特殊的结构。大厅，


 - Optimal MAPF solvers
  - Given this general state space, any A*-based algorithm can be used to solve the MAPF problem optimally.
  - b_base branching factor ：　每一个 vetex可以走的方向　基本上是４个方向
  - This paper focuses on 4-connected grids where b_base = 5, since every agent can move to the four cardinal directions or wait at its current location

- 3.3.1 Admissible heuristics for MAPF
  - non-trivial admissible heuristic for A*
    - A simple admissible heuristic is to sum the individual heuristics of the single agents such as Manhattan distance for 4-connected grids or Euclidean distance for Euclidean graphs [35]
  - HCA* improves on this by computing the optimal distance to the goal for each agent, ignoring other agents
  - he heuristic taken in the multi-agent A* search is the sum of these costs over all agents
  - This, however, must be recomputed for each problem instance with respect to each set of goal states
- 3.3.2 Drawbacks of A* for MAPF
 - 状态空间的大小与代理的数量 (k) 呈指数关系，这意味着 CLOSED 无法在内存中针对大型问题进行维护。
  - First, the size of the state space is exponential in the number of agents (k), meaning that CLOSED cannot be maintained in memory for large problems
 - Second, the branching factor of a given state may be exponential in k.

- 3.3.3 Reducing the effective number of agents with Independence Detecion
 - To this end Standley introduced the Independence Detection framework (ID) [45]. 
 - Note that ID is not perfect, in the sense that more independent subgroups may lay undetected in the groups returned by ID
 - Therefore, ID can be viewed as a general framework that utilizes an MAPF solver.
 - Indeed, in the experimental evaluation of CBS we ran ID on top of CBS.
 - ID 如何保证在求解完merged_group时，求解出来的path 和　其他的path 之间的conflict？

- 3.3.4 Enhancements to ID
 - Standley proposed a tie-breaking rule using a conflict avoidance table (CAT)
 - 求解merged_group时，增加CAT的约束，考虑冲突最少的方案
 - once two groups of agents are found to conflict, a resolve conflict procedure is called prior to merging the groups.
  - To maintain optimality, the cost of the plan found during the replanning must be exactly at the same cost as the original optimal solution for that group.

- 3.3.5 M*
 - related to ID
 - It is an A* based algorithm that dynamically changes the branching factor based on conflicts
 - q conflicting agents　make all possible moves and the k − q non-conflicting agents make their optimal move
 - RM* : divides the q conflicting agents into subgroups of agents each with independent conflicts
 - A variant called ODRM* [17] combines Standley’s Operator Decomposition (see Section 3.3.7) on top of RM* .

- 3.3.6 Avoiding surplus nodes
 - Under some restrictions, A* is known to expand the minimal number of nodes required to find an optimal solution [11]
 - Nodes with f > C ∗ , known as surplus nodes [15]
 - The challenge is how to identify surplus nodes during the search

- 3.3.7 Operator decomposion
 - OD -> 3.3.6
 - ?
 - Once the solution is found, intermediate nodes in OPEN are not developed further into regular nodes, so that the number of regular surplus nodes is significantly reduced 

- 3.3.8 Enhanced partial expansion
 - EPEA* [20] -> 3.3.6
 - to the best of our knowledge, is the best A* based solver for MAPF.
 - EPEA* 使用先验领域知识来避免产生多余节点 
 - generates only the children N_c with f(N_c)=f(N). This is done with the help of a domain-dependent operator selection function (OSF)
 - when using the SIC heuristic, the effect on the f -value of moving a single agent in a given direction can be efficiently computeds
 - Exact details of how this OSF for MAPF is computed and implemented can be found in [20]

- 3.3.9 The increasing cost tree search
 - the increasing cost tree search (ICTS) [40,42]
 - ICTS is a two-level search algorithm.
  - High level: At its high level, ICTS searches the increasing cost tree (ICT).
  - Low level: The low level acts as a goal test for the high level.
   - The task of the low level is to find a non-conflicting complete solution such that the cost of the individual path of agent a i is exactly C_i
  - If such a non-conflicting set of paths exists, the low level returns true and the search halts. Otherwise, false is returned and the high level continues to the next high-level node (of a different cost combination).
  - Pruning rules: Special pruning techniques were introduced for high-level nodes [42].
  - The enhanced version of ICTS (ICTS + pruning) [41,42] showed up to two orders of magnitude speedup over Standley’s A* approach (A* + OD + ID) [45].

# 综述归纳

- 集中式的求解MAPF的方法，主要分为三类：
 - Reduction-based solver (基于简化的求解器): 将MAPF问题简化为计算机科学的其他问题，然后求解
  - 典型的例子包括：SAT(Boolean Satisfiability)  ILP(Integer Linear Programming)  ASP(Answer Set Programming)
  - 特点：可以找到optimal solution, makeplan cost. 效率低甚至不适用在sum of cost funtion,只在小规模的例子中表现高效。需要你对其他简化的模型熟悉

- MAPF-specific sub-optimal solvers: 特定的MAPF次求解器
 - 特点：highly efficent (高效)，但是不能保证最优，是否可以完整求解
 - 主要的应用场景：agents比较大，最优解不是必须的。
 - 还可以继续划分类型
  - Search-based suboptimal solvers:基于搜索的次最优解法
   - HCA*
   - WHCA*
   - 特点

  - Rule-based suboptimal solvers:基于规则的次最优解法

  - Hybrid solvers: 混合的解法

- Optimal MAPF solvers: 最优的MAPF解法
  - A*
  - ID framework
  - EID
  - M*
  - Avoiding surplus nodes
  - OD
  - EPEA*
  - ICTS



# 问题：A* 算法中，node? 如何传播？


# 4 The conflict based search algorithm (CBS)
- The key idea of CBS is to grow a set of constraints and find paths that are consistent with these constraints.

## High level
- he high-level process of CBS and the search tree it searches
- 4.2.1 The constraint tree: a set of constraints, a solution, the total cost－f-value
- 4.2.1 Processing a node in the CT: low-level search for each egent, search paths find if has confict batween egents
- 4.2.3 Resolving a confict:
- 4.2.4 Conflict of k > 2 agents: second option
- 4.2.5 Edge conficts: (a_i, a_j, v1, v2, t)-> (a_i, v1, v2, t) constraint
- 4.2.6 Pseudo-code and example
 - The high level has the structure of a best-first search: OPEN容器模式？
## Low level: find paths for CT nodes
- A* handle the constraints: discard constraint (ai, v, t) -> (v, t)
- heuristic shortest path in the spatial dimension
- -same f-value: CAT

# 5 Theoretical analysis
- 5.1 optimality of CBS
- 5.2 Completeness of CBS
- 5.3 Comparision with other algorithms
 - 参数：Y　X　的意义？
- bottleneck   open spaces

# 6 CBS empirical evaluation
## 8 * 8 grid example
## DAO maps 

# 7 CBS using different cost functions 
## 7.1 High level
- ????

# Meta-agent conflict based(MA-CBS)


# 问题：　bottleneck指的是？MAO?: 瓶颈　grid
- SoC, makespan, and fuel cost functions: 误差函数

# 8 Meta-agent conflict based search (MA-CBS)

## 8.1 Motivation for meta-agent CBS
- CBS behaves poorly when a set of agents is strongly coupled

## 8.2 Mergung agents into a meta-agent

## 8.3 Merge policy
- Two agents a i , a j are merged into a meta-agent a { i , j } if the number of
conflicts between a i and a j recorded during the search exceeds a parameter B.
- If a conflict occurs between two meta-agents, a1 and a2 , because of two simple agents, a_t ∈ a1 and a_k ∈ a_2 , CM [t,k] is incremented by 1 and the shouldMerge ( ) function will return true if CM [x, y] > B over all x ∈ a1 , y ∈ a2

## 8.4 Merging constraints
- 8.4.1 Merging external constraints

# 9 MA-CBS experimental results
## 9.1 Conclusions from experiments
- MA-CBS with intermediate B values ( 0 < B < ∞) outperforms previous algorithm A*, EPEA* and CBS. It also outperforms ICTS in most cases.
- Density. In dense maps with many agents, low values of B are more efficient.
- Topology. In maps with large open spaces and few bottlenecks, low values of B are more efficient.
- Low-level solver. If a weak MAPF solver (e.g., plain A* ) is used for the low-level search, high values of B are preferred.

# Summary, conclusion, aand future work
- no heuristic guides the search inthe high-level constraint tree
- B parameter
- more policies than B
- experiment and compare different low-level solvers including  ICTS, ODrM* , SAT, ILP,ASP
- On a large scale, CSP  SAT

# Memory restricted CBS
 - 有一些内存方面的对比和建议

# 地图的特点
- top: large open space no bottlenecks
- middle: a few open spaces and a few bottlenecks
- bottom: almost no open spaces and many bottlenecks







