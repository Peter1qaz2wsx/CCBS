# 多机调度方案综述



## 集中式的求解MAPF的方法，主要分为三类：



### 1. Reduction-based solver (基于简化的求解器): 将MAPF问题简化为计算机科学的其他问题，然后求解

- 典型的例子包括：SAT(Boolean Satisfiability)  ILP(Integer Linear Programming)  ASP(Answer Set Programming)

- cost function: makeplan, sum of cost 效率低，不适用
- 特点：可以找到optimal solution, 只在小规模的例子中表现高效。需要你对其他简化的模型熟悉

### 2. MAPF-specific sub-optimal solvers: 特定的MAPF次求解器

特点：highly efficent (高效)，但是不能保证最优，是否可以完整求解, 主要的应用场景：agents比较大，最优解不是必须的。还可以继续划分类型

- Search-based suboptimal solvers:基于搜索的次最优解法

  - HCA* : close to optimal, not complete in many cases (Hierarchical Cooperative A*)

    - the agents are planned one at a time according to some predefined order

    - 前面找到的路径上的点，后面的egent不可用，应用了 global reservation table, 所以后面的egent可能没有路径

  - WHCA* : Windowed-HCA*. only performs cooperative pathfinding within a limited window

    - 主要指的是时间的窗口限制

  - 特点: agents too many deadlocks; not optimal; slow the search significatly 

- Rule-based suboptimal solvers:基于规则的次最优解法
  - different scenarious different specific movement rules.
  - 基于规则的求解器倾向于以低计算成本实现完整性而不是解决方案质量
  - run in polynomial time; TASS , Push and Swap
  - Both algorithms use a set of “macro” operators.
  - TASS and Push and Swap do not return an optimal solution and guarantee completeness for special cases only

- Hybrid solvers: 混合的解法
  - 这些方法对于一般情况并不完整，因为瓶颈可能会发生死锁
  - 这里介绍了一些混合的算法。主要包含三种：1,特定的运动规则以及重要的搜索 2, 预计算所有的路径，主要针对的是grids; 3, 基于搜索的算法，通过抽象减少状态空间的尺寸。将graph分解成多个subgraph,子图中包含特殊的结构, 如大厅等



### 3. Optimal MAPF solvers: 最优的MAPF解法

- A* : non-trivial admissible heuristic for A* .
  - this heuristic as the sum of individual costs heuristic (SIC)
  - 对于小的输入图，任何问题配置的SIC启发式算法都可以通过预先计算输入图g的全对最短路径矩阵来存储为查找表。对于大的图，我们只计算从每个状态到目标状态的最短路径
  - draw_back: 状态空间的大小与代理的数量 (k) 呈指数关系，这意味着 CLOSED 无法在内存中针对大型问题进行维护;
  -  the branching factor of a given state may be exponential in k.

- ID framework: Independence Detecion

    - Reducing the effective number of agents with Independence Detecion

    - 首先是，所有的egents应用Ａ* 独立找到自己的path, 然后检查各个路径之间是否存在冲突，如果存在，merge egents. 重复直到没有

    - 缺点: the sense that more independent subgroups may lay undetected in the groups returned by ID.

- EID: improve the chance of identifying independent groups of agents

    - The paths that were found for the agents are stored in the CAT

    - 求解merged_group时，增加CAT的约束，考虑冲突最少的方案

    - once two groups of agents are found to conflict, a resolve conflict procedure is called prior to merging the groups.

    - the cost of the plan found during the replanning must be exactly at the same cost as the original optimal solution for that group

- M*

    - It is an A* based algorithm that dynamically changes the branching factor based on conflicts

    - 一般来说，扩展节点只生成一个子节点，每个agent朝着目标进行最优移动。这种情况一直持续到节点n的q≥2个代理之间发生冲突。在这种情况下，需要局部增加搜索维数。M* 从n开始回溯到n的所有祖先节点，直到根节点，所有这些节点都被放回到OPEN中。如果这些节点中的一个再次展开，它将生成b q个子节点，其中q个冲突代理做出所有可能的移动，k−q个非冲突代理做出最优移动。

    - An enhanced version, called Recursive M* (RM*) [51] divides the q conflicting agents into subgroups of agents each with independent conflicts

    - A variant called ODRM* [17] combines Standley’s Operator Decomposition (see Section 3.3.7) on top of RM*.

- Avoiding surplus nodes

    - The challenge is how to identify surplus nodes during the search 

- OD: Operator decomposion

    - Agents are assigned an arbitrary (but fixed) order.

    - 代理被分配一个任意(但固定)的顺序。当一个普通的a* 节点展开时，OD只考虑并应用第一个代理的移动。这样做会引入一个中间节点。在中间节点，只考虑单个代理的移动，生成更多的中间节点。当一个操作符应用到最后一个代理时，将生成一个常规节点。一旦找到解决方案，OPEN中的中间节点就不会进一步发展为规则节点，从而大大减少了规则剩余节点的数量。

- EPEA* : Enhanced partial expansion

    - to the best of our knowledge, is the best A* based solver for MAPF.

    - EPEA* 使用先验领域知识来避免产生多余节点 

    - generates only the children N_c with f(N_c)=f(N). This is done with the help of a domain-dependent operator selection function (OSF)

    - when using the SIC heuristic, the effect on the f-value of moving a single agent in a given direction can be efficiently computed

- ICTS: the increasing cost tree search

    - ICTS is a two-level search algorithm.

    - High level: At its high level, ICTS searches the increasing cost tree (ICT). a breadth-first search of the ICT will find the optimal solution

    - Low level: The low level acts as a goal test for the high level. 

    - The task of the low level is to find a non-conflicting complete solution such that the cost of the individual path of agent a i is exactly C_i

    - If such a non-conflicting set of paths exists, the low level returns true and the search halts. Otherwise, false is returned and the high level continues to the next high-level node (of a different cost combination).

    - Pruning rules: Special pruning techniques were introduced for high-level nodes [42].

    - The enhanced version of ICTS (ICTS + pruning) [41,42] showed up to two orders of magnitude speedup over Standley’s A* approach (A* + OD + ID) [45].

      

## 暂时考虑的方面

- 这里是的多机调度可以认为是在local区域内，当egents之间可以通信时的调度

  

## 可以重点关注算法

- ID,EID: merged framework

- EPEA* : is the best A* based solver for MAPF

- ICTS, 也可以应用在low-level

- CBS: 

  - high-level
  - low-level

- SIPP: CCBS中low-level用的算法

- 算法的代码仓库：http://mapf.info/index.php/Main/Software

