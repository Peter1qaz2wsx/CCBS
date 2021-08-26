## 代码分析



- 代码分析

  ```c++
  // agent: 在该agent下增加constraint; move1: agent对应的move会存在冲突。move2: 另外一个agent的ｍove２和当前的move1产生冲突
  Constraint CBS::get_constraint(int agent, Move move1, Move move2)
  {
      // 如果move1的起始位置和结束位置相等，表示是wait的约束
      if(move1.id1 == move1.id2)
          return get_wait_constraint(agent, move1, move2);
      // A表示agent的初始位置，A2表示agent的目标位置；Ｂ表示了另一个agent的初始位置，B2表示另一个agent的目标位置
      double startTimeA(move1.t1), endTimeA(move1.t2);
      Vector2D A(map->get_i(move1.id1), map->get_j(move1.id1)), A2(map->get_i(move1.id2), map->get_j(move1.id2)),
               B(map->get_i(move2.id1), map->get_j(move2.id1)), B2(map->get_i(move2.id2), map->get_j(move2.id2));
      // 如果mov2.t2等于无穷表示的意思？
      if(move2.t2 == CN_INFINITY)
          // 返回一个约束：agent 从id1到id2在时间t1之后，不可通行
          return Constraint(agent, move1.t1, CN_INFINITY, move1.id1, move1.id2);
      double delta = move2.t2 - move1.t1;
      // while的目的是获取一个时间的上限，在（startTimeA, 上限时间）时间区间，不允许move1的运动
      while(delta > config.precision/2.0)
      {
          // 检查两个move是否存在冲突，如果存在冲突
          if(check_conflict(move1, move2))
          {
              // move.t1等于move2.t2，最大的时间上限是move2.t2
              move1.t1 += delta;
              move1.t2 += delta;
          }
          //　没有冲突
          else
          {
              // 如果没有冲突，减少时间上限。二分法
              move1.t1 -= delta;
              move1.t2 -= delta;
          }
          if(move1.t1 > move2.t2 + CN_EPSILON)
          {
              // 最大的时间上限是move2.t2
              move1.t1 = move2.t2;
              move1.t2 = move1.t1 + endTimeA - startTimeA;
              break;
          }
          delta /= 2.0;
      }
      if(delta < config.precision/2.0 + CN_EPSILON && check_conflict(move1, move2))
      {
          move1.t1 = fmin(move1.t1 + delta*2, move2.t2);
          move1.t2 = move1.t1 + endTimeA - startTimeA;
      }
      return Constraint(agent, startTimeA, move1.t1, move1.id1, move1.id2);
  }
  
  // 获取wait的约束
  Constraint CBS::get_wait_constraint(int agent, Move move1, Move move2)
  {
      double radius = 2*config.agent_size;
      // i0，i1 分别为 move2的起始点位置，目标位置
     // i2 为move1等待的位置
      double i0(map->get_i(move2.id1)), j0(map->get_j(move2.id1)), i1(map->get_i(move2.id2)), j1(map->get_j(move2.id2)), i2(map->get_i(move1.id1)), j2(map->get_j(move1.id1));
      std::pair<double,double> interval;　
      Point point(i2,j2), p0(i0,j0), p1(i1,j1);
      // classify的作用：分析 wait的目标点和move2之间的关系
      // 在loadmap的情况下，p2和p1相等，也就是cls=6
      int cls = point.classify(p0, p1);
      //　dist：Point到向量p0，p1的距离，计算有问题 dist = p0point 叉乘 p0p1 / p0p1的距离
      double dist = fabs((i0 - i1)*j2 + (j1 - j0)*i2 + (j0*i1 - i0*j1))/sqrt(pow(i0 - i1, 2) + pow(j0 - j1, 2));
      // da 表示point到p0之间的欧式距离
      double da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
      // db 表示point到p1的欧式距离
      double db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);
      // ha 表示p在p0p1上的投影点到p0之间的距离
      double ha = sqrt(da - dist*dist);
      // size 表示两个车之间的距离和最近距离之间的差值
      double size = sqrt(radius*radius - dist*dist);
      if(cls == 3)
      {
          interval.first = move2.t1;
          interval.second = move2.t1 + (sqrt(radius*radius - dist*dist) - ha);
      }
      else if(cls == 4)
      {
          interval.first = move2.t2 - sqrt(radius*radius - dist*dist) + sqrt(db - dist*dist);
          interval.second = move2.t2;
      }
      else if(da < radius*radius)
      {
          if(db < radius*radius)
          {
              interval.first = move2.t1;
              interval.second = move2.t2;
          }
          else
          {
              double hb = sqrt(db - dist*dist);
              interval.first = move2.t1;
              interval.second = move2.t2 - hb + size;
          }
      }
      else
      {
          if(db < radius*radius)
          {
              interval.first = move2.t1 + ha - size;
              interval.second = move2.t2;
          }
          else
          {
              interval.first = move2.t1 + ha - size;
              interval.second = move2.t1 + ha + size;
          }
      }
      return Constraint(agent, interval.first, interval.second, move1.id1, move1.id2);
  }
  
  // Point结构体
  class Point {
  public:
      double i;
      double j;
  
      Point(double _i = 0.0, double _j = 0.0):i (_i), j (_j){}
      Point operator-(Point &p){return Point(i - p.i, j - p.j);}
      int operator== (Point &p){return (i == p.i) && (j == p.j);}
      // 作用：
      int classify(Point &pO, Point &p1)
      {
          Point p2 = *this;
          Point a = p1 - pO;
          Point b = p2 - pO;
          // 二维向量的叉乘
          // 叉乘的结果是正数，说明a到b是逆时针，反之顺时针；结果若是0，则说明a，b共线。
          // 分这么多主要是针对网格地图的
          double sa = a.i * b.j - b.i * a.j;
          if (sa > 0.0)
              return 1;//LEFT;
          if (sa < 0.0)
              return 2;//RIGHT;
          if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0))
              return 3;//BEHIND;
          if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j))
              return 4;//BEYOND;
          if (pO == p2)
              return 5;//ORIGIN;
          if (p1 == p2)
              return 6;//DESTINATION;
          return 7;//BETWEEN;
      }
  };
  
  ```
  
  