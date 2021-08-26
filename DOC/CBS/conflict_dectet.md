# 冲突检测理解

## 代码理解

``` c++
// 检查两个move是否存在冲突 
bool CBS::check_conflict(Move move1, Move move2)
{
    double startTimeA(move1.t1), endTimeA(move1.t2), startTimeB(move2.t1), endTimeB(move2.t2);
    int m1i1(map->get_i(move1.id1)), m1i2(map->get_i(move1.id2)), m1j1(map->get_j(move1.id1)), m1j2(map->get_j(move1.id2));
    int m2i1(map->get_i(move2.id1)), m2i2(map->get_i(move2.id2)), m2j1(map->get_j(move2.id1)), m2j2(map->get_j(move2.id2));
     // Ａ,B 分别为两个move的起始位置，VA ，VB分别表示agent的速度
    Vector2D A(m1i1, m1j1);
    Vector2D B(m2i1, m2j1);
    Vector2D VA((m1i2 - m1i1)/(move1.t2 - move1.t1), (m1j2 - m1j1)/(move1.t2 - move1.t1));
    Vector2D VB((m2i2 - m2i1)/(move2.t2 - move2.t1), (m2j2 - m2j1)/(move2.t2 - move2.t1));
    // 如果move2开始的时间比move1晚
    if(startTimeB > startTimeA)
    {
        // 计算在move2开始移动时，move1的位置，move1开始时间变为startTimeB
        A += VA*(startTimeB-startTimeA);
        startTimeA = startTimeB;
    }
    //  如果move1开始的时间比move2晚
    else if(startTimeB < startTimeA)
    {
        // 计算在move1开始移动时，move2的位置，move2开始时间变为startTimeA
        B += VB*(startTimeA - startTimeB);
        startTimeB = startTimeA;
    }
    // 机器人的直径
    double r(2*CN_AGENT_SIZE);
    // move1, move2 起始位置的差距
    Vector2D w(B - A);
    // 计算两个机器人除掉几何外形之后的欧氏距离
    double c(w*w - r*r);
    // 如果两个车之间的距离小于车体的几何大小，存在冲突
    if(c < 0)
        return true;
	// v 表示两个机器人之间的速度差
    Vector2D v(VA - VB);
    // a表示速度向量的欧式距离值
    double a(v*v);
    // ｂ 表示
    double b(w*v);
    double dscr(b*b - a*c);
    // 方程无解dscr 小于或则等于0
    if(dscr - CN_EPSILON < 0)
        return false;
    // ctime 碰撞的时间解
    double ctime = (b - sqrt(dscr))/a;
    if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON)
        return true;
    return false;
}
```

## 理论基础

- This means that each animated character a user may encounter in the current scene, no matter how complex, is described by a simple abstract representation known as an agent.

- 这意味着用户在当前场景中可能遇到的每个动画角色，无论多么复杂，都由称为代理的简单抽象表示进行描述。 

- Each agent has a few variables that store the state of the corresponding animated character

- 每个代理都有一些变量，用于存储相应动画角色的状态 

- The exact variables used will vary from different implementations, but common agent states include position, velocity, radius, and goal velocity.

- 使用的确切变量因不同的实现而异，但常见的代理状态包括位置、速度、半径和目标速度。 

- Agent State

  - ![](/home/liukang/文档/20210701/CBS/选区_013.png)

  - the radius of the disc defines the area that is occupied by the agent and that other agents cannot step into.
  - Position (2D float vector): A simple 2D vector of the agent’s x and y position is needed to locate the agent.
  - Velocity (2D float vector): The agent moves across the virtual world with a certain velocity.
  - Goal velocity (2D float vector): At any time instant, the agent prefers to move toward a certain direction at a certain given speed.

- Predicting Collisions (Time to Collision)

  - We can use the concept of a time to collision (denoted τ) to reason about upcoming interactions.

  - Specifically, a collision between two agents is said to occur at some time τ ≥ 0, if the corresponding discs of the agents intersect

  - the problem can be simplified into computing the distance between the extrapolated positions of the agents and comparing it against the sum of the combined radii of the agents.

  - $$
    \begin{Vmatrix} (x_B + v_B\tau) - (x_A  + v_A\tau)\end{Vmatrix} = r_A + r_B
    $$

  - $$
    (v \cdot v)\tau^2 + 2(w\cdot v)\tau + w\cdot w - (r_A + r_B)^2 =0
    $$

  - $$
    w = x_B - x_B
    $$

    $$
    v = v_B - v_A
    $$

    

    

  - If both solutions are negative, then no collision takes place and τ is undefined.

  - If one solution is negative and the other is nonnegative, then the agents are currently colliding, that is, τ = 0.

  - If both solutions are nonnegative, then a collision occurs at τ = min( τ + , τ − ).

  - Assuming that the agents are not currently colliding, it suffices to test whether τ − is nonnegative. Otherwise, τ is undefined.
