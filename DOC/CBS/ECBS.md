ECBS:  

- ECBS is an extension of CBS. It uses the same two-level search with a focal search on both levels.

- The focal search uses a second user-provided inadmissible heuristic that minimizes conflicts between agents.

- Greedy-CBS (GCBS): Suboptimal CBS
  - Relaxing the High-Level: The main idea in GCBS is to prioritize CT nodes that seem closer to a goal node (in terms of depth in the CT, AKA distance-to-go (Thayer and Ruml 2011)). 
  - The high-level in GCBS favors nodes with minimal conflict heuristic, i.e., it chooses the node with minimal h_c.

- Bounded Suboptimal CBS
  - We thus use Focal search, an alternative approach to obtain bounded suboptimal search algorithms, based on the A∗ (Pearl and Kim 1982) and A (Ghallab and Allard 1983) algorithms.
  - Focal search maintains two lists of nodes: OPEN and FOCAL. OPEN is the regular OPEN-list of A*. FOCAL contains a subset of nodes from OPEN.
  - Focal search: f1  f2 function, f1 defines which nodes in FOCAL.  Given a suboptimality factor w, FOCAL contains all nodes n in OPEN for which f1(n) ≤ w ·f1min . f2 is used to choose which node from FOCAL to expand. 
  - If f1 is admissible then we are guaranteed that the returned solution is at most w*
  
- BCBS

  - To obtain a bounded suboptimal variant of CBS we can implement both levels of CBS as a focal search
  - High level focal search: 
    -  apply focal-search(g,hc) to search the CT, where g(n) is the cost of the CT node n, and hc(n) is the conflict heuristic described above.
  - Low level focal search:
    - apply focal-search(f,hc) to find a consistent single agent path, where f(n) is the regular f(n) = g(n) + h(n) of A*, and hc(n) is the conflict heuristic described above, considering the partial path up to node n for the agent that the low level is planning for.

- Enhanced CBS

  - How to distribute w between wH and wL is not trivial. The best performing distribution is domain dependent
  - CBS runs the same low level search as BCBS(1,w)
  - he advantage of ECBS over BCBS is that while allowing the low level the same flexibility as BCBS(1,w), it provides additional flexibility in the high level when the low level finds low cost solutions (i.e, when LB(n) is close to n.cost).

- 总结：

  high_level 和 Low-level 都使用“optimal best-first search”，使用 "OPEN" "FOCAL"容器来进行分支和搜索路径。算法的主要侧重点都集中在搜索下一个节点的指导项上，增加了两个指导函数f1和f2. f1决定在OPEN中的那些节点加入FOCAL中，f2决定那个分支expand。在Focal search中增加了一个w参数，作为f1的边界点参数，控制FOCAL中的node.

  ECBS:  LOW-LEVEL 和之前的BBS一样，HIGH-LEVEWL根据low-level返回的数据确认
  $$
  FOCAL = \lbrace n|n\ \in  OPEN, n.cost \le LB \cdot w
  $$
  

## 代码框架梳理

首先代码的结构不是很有调理："agent"的输入和输出不是同一次的输入。

high-level: 主要过程是增加了一个FOCAL容器，通过每一次分支 得到的cost小于分支之前的ｗ倍（f*w),

每一次从FOCAL中取LB值最小的分支，如果得到没有冲突的路径就可以得到结果

low-level:  LB的计算path.fmin

```c++
// 初始化low-level。输入参数地图信息, 约束信息，solution？作用是？
LowLevelEnvironment llenv(m_env, i, newNode.constraints[i],
                                  newNode.solution);
//  定义搜索的方法，加入w信息
        LowLevelSearch_t lowLevel(llenv, m_w);
// low-level搜索路径
        bool success = lowLevel.search(initialStates[i], newNode.solution[i]);

```

计算LB的算法：后面看