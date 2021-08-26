# Extended Increasing Cost Tree Search for Non-Unit Cost Domains

- In this paper we introduce a new definition of the MAPF problem for non-unit cost and non-unit time step domains along with new multi-agent state successor generation schemes for these domains.

- we define an extended version of the increasing cost tree search algorithm (ICTS) for non-unit costs, with two new sub-optimal variants of ICTS: -ICTS and w-ICTS.
- Our experiments show that higher quality sub-optimal solutions are achievable in domains with finely discretized movement models in no more time than lower-quality, optimal solutions in domains with coarsely discretized movement models.



## Experimental Results and Analysis

- OD-Style Versus Full Branching
- ICTS Versus A* and CBS
  - Both A* and ICTS use OD-style branching and the ID framework, solving only conflicting agents jointly. The CBS algorithm is using continuous-time collision detection and the conflict prioritization (PC) enhancement from ICBS
  - ![](/home/liukang/文档/20210701/CBS/选区_017.png)
  - 在4连通的区域CBS和ICTS 差不多，８和16连通，CBS明显不好