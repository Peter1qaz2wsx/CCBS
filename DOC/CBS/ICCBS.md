# Improving Continuous-time Conflict Based Search*

## Heuristics for High-Level Search

- These heuristics estimate the difference in cost between a CT node and the optimal solution
- 这些启发式方法估计了 CT 节点与最优解之间的cost差异 
- they are a lower bound on the actual cost difference, and therefore can be safely added to the cost of a CT node when choosing which node to expand next
- 它们是实际成本差异的下限，因此在选择下一步扩展哪个节点时可以安全地添加到 CT 节点的成本中 
- Indeed, these heuristics were shown to significantly decrease the number of the expanded CT nodes and improve the performance of CBS.
- 事实上，这些启发式方法被证明可以显着减少扩展 CT 节点的数量并提高 CBS 的性能。 
- two admissible heuristics for CCBS
- The first admissible heuristic, denoted H1, is based on solving the following linear programming problem (LPP) .  基于解决线性规划问题
- Each conflict( Con i,j) between agents i and j in the CT node for which we are computing the heuristic introduces the LPP constraint P x i + x j ≥∆(Con i,j )
- By construction, for any solution to this LPP, the value i=1 x i is an admissible heuristic since for every conflict Con i,j the solution cost is increased by at least ∆(Con i,j ).

- second:  There, the heuristic was based on identifying disjoint cardinal conflicts, which are cardinal conflicts between disjoint pairs of agents
- Therefore, in the H2 heuristic we aim to choose the disjoint cardinal conflicts that would have the largest cost impact.
- The H2 heuristic is the sum of the cost impacts of the chosen conflicts



- 总结：本文为每一个node定义的启发项是在特定的主conflict下对应的启发项。

## Disjoint Splitting for CCBS

- DS is a technique designed to ensure that expanding a CT node N creates a disjoint partition of the space of solutions that satisfy the constraints in N.constraints.
- To address this inefficiency, CBS with DS (CBS-DS) introduces the notion of positive and negative constraints.
- 主要的思路，在子节点中增加 positive constraints. 也就是在一个节点中增加必须通过的点，也就是landmarks，这样可以减少 low-level搜索



## PC: Prioritizing Conflicts

- 如何选择要解决的冲突，会造成CT的尺寸差异，从而造成运行时间差异。

  cardinal：主要冲突，两个子节点的cost高于父节点

  semi-cardinal：次要冲突，有一个子节点高于父节点

  non-cardinal：以上都不是

- cost impact：解决冲突时候，solution cost增加了多少

