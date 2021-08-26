## 分析ccbs相应的参数

- use_cardinal: true  use_disjoint_splitting: true  hlh_type: 2

  ``` mathematica
  Soulution found: true　 是否找到了最优解
  Runtime: 0.0110817　　消耗的总时间
  Makespan: 282.4　　　  最长的路径消耗cost
  Flowtime: 1927.01　　   所有的累计cost
  Initial Cost: 1903.41　　初始话时所有的累计cost
  Collision Checking Time: 0.00681249    冲突检测消耗的时间
  HL expanded: 158　　　　　　　　　　high-level 的层数
  LL searches: 862　　　　　　　　　　   low-level的运行的次数
  LL expanded(avg): 9.03016　　　　　　low-level单次搜索的平均次数
  
  ```

- use_cardinal: false  use_disjoint_splitting: true  hlh_type: 2

  ```mathematica
  Soulution found: true
  Runtime: 0.0808797
  Makespan: 282.4
  Flowtime: 1927.01
  Initial Cost: 1903.41
  Collision Checking Time: 0.0184122
  HL expanded: 1981
  LL searches: 3960
  LL expanded(avg): 8.51035
  
  ```

  - 时间明显增加，节点数量明显增加，

- use_cardinal: false  use_disjoint_splitting: false  hlh_type: 2

  ```mathematica
  Soulution found: false
  Runtime: 30
  Makespan: 287.842
  Flowtime: 1925.79
  Initial Cost: 1903.41
  Collision Checking Time: 6.67414
  HL expanded: 922981
  LL searches: 1845960
  LL expanded(avg): 13.0841
  ```

  - 无法找到结果

- use_cardinal: true  use_disjoint_splitting: false  hlh_type: 2

  ```mathematica
  Soulution found: true
  Runtime: 0.0393868
  Makespan: 282.4
  Flowtime: 1927.01
  Initial Cost: 1903.41
  Collision Checking Time: 0.026665
  HL expanded: 469
  LL searches: 3074
  LL expanded(avg): 11.8058
  ```

  - HL 增加一部分。LL 增加很多。
  - 说明：use_cardinal　减少了很多节点的搜索

- use_cardinal: true  use_disjoint_splitting: false  hlh_type: 1 

  - hlh_type: 1 利用　simplex.set_problem(coefficients, overcosts); 线性方程求解

  ```mathematica
  Soulution found: true
  Runtime: 0.0155444
  Makespan: 282.4
  Flowtime: 1927.01
  Initial Cost: 1903.41
  Collision Checking Time: 0.00720734
  HL expanded: 158
  LL searches: 862
  LL expanded(avg): 9.03016
  
  ```

  - 时间有很少的增加，但是求解方程应该存在内存的消耗，同事还需要适配 simplx库

- use_cardinal: true  use_disjoint_splitting: false  hlh_type: ０　

  - hlh_type: ０不增加新的启发项

  ```mathematica
  Soulution found: true
  Runtime: 0.0220332
  Makespan: 282.4
  Flowtime: 1927.01
  Initial Cost: 1903.41
  Collision Checking Time: 0.0134524
  HL expanded: 228
  LL searches: 1192
  LL expanded(avg): 8.88591
  ```

  - 时间有一倍左右的增加，冲突检测花费时间多，节点也有一定的增加

## 需要理清楚的参数

- map 中节点的坐标是什么单位？查看代码中好像是已经转化成时间相关的参数了？

  cost 计算的是欧式距离

  代码哪里写的是时间？在low-level搜索时，用的是时间相关的cost?

  查看代码：cbs中Node.interval的计算　“get_wait_constraint（）”函数

  Move(Node a, Node b) : t1(a.g), t2(b.g), id1(a.id), id2(b.id) {}：时间概念用的是欧式距离。距离是每一个step的累计的和。

  时间t用的是欧式距离替代的。相当于速度是１m/s ?

  

## 搜索路径变慢分析

agent: 半径6

- Soulution found: true
  Runtime: 0.0167291
  Makespan: 310.82
  Flowtime: 636.873
  Initial Cost: 491.95
  Collision Checking Time: 0.00953522
  HL expanded: 108
  LL searches: 556
  LL expanded(avg): 5.28957

agent: 半径 0.353553

- Soulution found: true
  Runtime: 3.46298
  Makespan: 310.82
  Flowtime: 636.873
  Initial Cost: 491.95
  Collision Checking Time: 1.74558
  HL expanded: 74982
  LL searches: 374160
  LL expanded(avg): 5.40794

其他参数的影响：

- use_cardinal = false

  - Soulution found: true
    Runtime: 2.26167
    Makespan: 310.82
    Flowtime: 636.873
    Initial Cost: 491.95
    Collision Checking Time: 0.185468
    HL expanded: 91552
    LL searches: 183102

    LL expanded(avg): 5.23303

- use_disjoint_splitting = false

  - Soulution found: false
    Runtime: 30
    Makespan: 321.7
    Flowtime: 512.031
    Initial Cost: 491.95
    Collision Checking Time: 17.7209
    HL expanded: 786792
    LL searches: 4392090
    LL expanded(avg): 8.1191

- hlh_type =0

  - Soulution found: true
    Runtime: 4.27917
    Makespan: 310.82
    Flowtime: 636.873
    Initial Cost: 491.95
    Collision Checking Time: 2.11468
    HL expanded: 91551
    LL searches: 443834
    LL expanded(avg): 5.33412

- 影响因子：

  - 初始化一样，不影响

  - agent_size :  越大减少的越多

    - agent 变大影响的原因
      - 半径参数应用的地方：
      - check_conflict　计算是否存在confict
      - **get_wait_constraint**　中

  - use_cardinal:    Collision Checking Time时间减少了1.6s, 

    HL expanded 和LL expanded数量分别增加

    74982　　　91552         

    374160　　183102

    use_cardinal 等于true时，find_new_conflicts函数会搜索路径，目的是更新主要次要冲突

    

  - Collision Checking Time:  统计的时间包括
  
    - 第一个：　获取优先需要解决的冲突
    - 第二个：　获取右子节点的新的conflict　
    - 第三个：　获取左子节点的新的conflict
    - 第二和第三中包含LL路径搜索
    - 严格来说还是　node太多影响的

## 半径为什么会影响HL中node的数量

- check_conflict()函数的影响
  - 函数的主要作用：判断两个移动是否存在冲突
  - check_paths函数：主要查看两个路径之间是否存在冲突，存在冲突时，返回对应的冲突
  - get_all_conflicts函数：获取对应的agent中存在的冲突
  - find_new_conflicts函数：新的node，增加了新的constraint。查找新的冲突
- get_wait_constraint()函数
  - 函数的主要作用：获取等待的约束
- 可能的原因：机器人半径影响了约束中的非安全时间间隔的大小，当半径大时，安全间隔的时间大，在一定程度上对于SIPP算法搜索路径产生一定的影响



## 分析agent 半径

半径：　６

9 ,  10 ,  61.0164 ,  74.6662

4 ,  10 ,  72.4611 ,  82.8051

9 ,  10 ,  61.0164 ,  74.6662

4 ,  10 ,  72.4611 ,  82.8051

4 ,  10 ,  82.8051 ,  166.987

10 ,  4 ,  118.884 ,  130.907

10 ,  4 ,  118.884 ,  130.907

4 ,  10 ,  82.8051 ,  166.987

4 ,  10 ,  82.8051 ,  94.8281

10 ,  4 ,  118.884 ,  130.907

4 ,  10 ,  82.8051 ,  94.8281

4 ,  10 ,  94.8281 ,  179.01

10 ,  4 ,  130.907 ,  142.93

9 ,  10 ,  73.0393 ,  85.0103

4 ,  10 ,  82.8051 ,  94.8281

4 ,  10 ,  82.8051 ,  94.8281

9 ,  10 ,  73.0393 ,  85.0103

4 ,  10 ,  94.8281 ,  179.01

10 ,  4 ,  130.907 ,  142.93

4 ,  10 ,  94.7761 ,  106.799

半径：0.353553

10 ,  4 ,  118.884 ,  120.563

4 ,  10 ,  72.4611 ,  166.987

10 ,  4 ,  118.884 ,  120.563

4 ,  10 ,  72.4611 ,  166.987

4 ,  10 ,  72.4611 ,  73.1695

10 ,  4 ,  118.884 ,  119.593

4 ,  10 ,  72.4611 ,  73.1695

4 ,  10 ,  73.1695 ,  168.666

10 ,  4 ,  120.563 ,  121.272

9 ,  10 ,  62.6953 ,  63.4007

4 ,  10 ,  72.4611 ,  73.1695

4 ,  10 ,  72.4611 ,  73.1695

9 ,  10 ,  62.6953 ,  63.4007

4 ,  10 ,  73.1695 ,  168.666

10 ,  4 ,  120.563 ,  121.272

4 ,  10 ,  73.1665 ,  73.8749

9 ,  10 ,  63.4007 ,  64.106
