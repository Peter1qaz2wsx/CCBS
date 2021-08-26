#include "sipp.h"

void SIPP::clear()
{
    open.clear();
    close.clear();
    collision_intervals.clear();
    landmarks.clear();
    constraints.clear();
    visited.clear();
    path.cost = -1;
}

double SIPP::dist(const Node& a, const Node& b)
{
    return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2));
}

/*
    find_successors函数主要的功能获取当前node的有效的下一步move的位置newNode,newNode还包括不同的安全时间间隔相同位置的node。
    更新newNode的优先级代价f(n)。其中f(n)是起始点到当前的goalNode的优先级代价. 当前有效的newＮode插入到visited中或则更新更小的g。
    并且把有效的newNode存储在succs容器中，同时飞最小的node不要。
    传入的参数goal用来判断当前的goal是不是最终的agent_goal，处理不同的f的值。同时也用来更新h的值。
*/
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
        //　计算当前node到相邻的node之间的欧式距离
        double cost = dist(curNode, newNode);
        // 当前node到起始位置的距离 curNode.g, 更新新的node到起始位置的距离
        newNode.g = curNode.g + cost;
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
        auto cons_it = constraints.find({curNode.id, newNode.id});
        int id(0);
        // 遍历可用间隔时间
        for(auto interval: intervals)
        {
            // id 可以标记newNode中可用时间间隔
            newNode.interval_id = id;
            id++;
            // 查看当前节点在当前时间间隔内是否已经存在visited节点了
            auto it = visited.find(newNode.id + newNode.interval_id * map.get_size());
            if(it != visited.end())
                // 如果存在，并且已经在路径上了，直接跳过
                if(it->second.second)
                    continue;
            // 如果当前的安全时间的上限小于起始位置到当前的时间，说明机器人到达当前node的最小时间大于安全时间间隔上限，直接跳过
            if(interval.second < newNode.g)
                continue;
            // 如果当前的安全时间的下限大于起始位置到当前的时间，newNode.g可以用interval.first;时间替换
            if(interval.first > newNode.g)
                newNode.g = interval.first;
            //　如果curNode.id, newNode.id存在不可通行的时间约束
            if(cons_it != constraints.end())
                //　遍历约束
                for(unsigned int i = 0; i < cons_it->second.size(); i++)
                    // 如果在curNode时的时间在约束范围内，跳过这个范围
                    if(newNode.g - cost + CN_EPSILON > cons_it->second[i].t1 && newNode.g - cost < cons_it->second[i].t2)
                        newNode.g = cons_it->second[i].t2 + cost;
            newNode.interval = interval;
            // 如果 newNode.g - cost 不在当前的时间安全间隔内，或者newNode.g不在安全时间间隔内，直接跳过
            if(newNode.g - cost > curNode.interval.second || newNode.g > newNode.interval.second)
                continue;
            // 如果 当前节点在当前时间间隔内已经存在visited节点了,但是没有在路径node上
            if(it != visited.end())
            {
                //　已存在的点的cost更新，跳过。找最小的cost
                if(it->second.first - CN_EPSILON < newNode.g)
                    continue;
                else
                    it->second.first = newNode.g;
            }
            else
                // 如果不存在，直接把当前的节点添加到visited中
                visited.insert({newNode.id + newNode.interval_id * map.get_size(), {newNode.g, false}});
            // 如果当前的goal站点是最终的目标站点，计算启发函数值f：当前的cost加上启发项
            if(goal.id == agent.goal_id) //perfect heuristic is known
                newNode.f = newNode.g + h_values.get_value(newNode.id, agent.id);
            else
            {
                // 如果当前的goal站点不是最终的目标站点，重新计算欧式距离作为启发项的值
                double h = sqrt(pow(goal.i - newNode.i, 2) + pow(goal.j - newNode.j, 2));
                //通过启发项表计算最大的启发项值。计算
                for(unsigned int i = 0; i < h_values.get_size(); i++) //differential heuristic with pivots placed to agents goals
                    h = std::max(h, fabs(h_values.get_value(newNode.id, i) - h_values.get_value(goal.id, i)));
                newNode.f = newNode.g + h;
            }
            succs.push_back(newNode);
        }
    }
}

Node SIPP::find_min()
{
    Node min = *open.begin();
    open.pop_front();
    return min;
}

void SIPP::add_open(Node newNode)
{
    // open现在是空，或者最后一个node的优先级代价小于当前的代价，直接放入容器
    if (open.empty() || open.back().f - CN_EPSILON < newNode.f)
    {
        open.push_back(newNode);
        return;
    }
    for(auto iter = open.begin(); iter != open.end(); ++iter)
    {
        if(iter->f > newNode.f + CN_EPSILON) // if newNode.f has lower f-value
        {
            open.insert(iter, newNode);
            return;
        }
        else if(fabs(iter->f - newNode.f) < CN_EPSILON && newNode.g + CN_EPSILON > iter->g) // if f-values are equal, compare g-values
        {
            open.insert(iter, newNode);
            return;
        }
    }
    open.push_back(newNode);
    return;
}

// reconstruct_path函数的作用：返回一个path路径，一系列的node
std::vector<Node> SIPP::reconstruct_path(Node curNode)
{
    path.nodes.clear();
    // 将路径上的node插入到  path.nodes中
    if(curNode.parent != nullptr)
    do
    {
        path.nodes.insert(path.nodes.begin(), curNode);
        curNode = *curNode.parent;
    }
    while(curNode.parent != nullptr);
    //把最开始的node插入
    path.nodes.insert(path.nodes.begin(), curNode);
    for(unsigned int i = 0; i < path.nodes.size(); i++)
    {
        unsigned int j = i + 1;
        if(j == path.nodes.size())
            break;
        // 如果相邻的两个node的cost大于欧式距离，表示：增加一个等待node
        if(fabs(path.nodes[j].g - path.nodes[i].g - dist(path.nodes[j], path.nodes[i])) > CN_EPSILON)
        {
            Node add = path.nodes[i];
            add.g = path.nodes[j].g - dist(path.nodes[j], path.nodes[i]);
            // 增加一个新的node，和前面的node形成等待的行为
            path.nodes.emplace(path.nodes.begin() + j, add);
        }
    }
    return path.nodes;
}

 // positive = false, 如果是wait，那么作为冲突间隔. 同时如果存在有交集的时间间隔，合并冲突时间
void SIPP::add_collision_interval(int id, std::pair<double, double> interval)
{
    std::vector<std::pair<double, double>> intervals(0);
    if(collision_intervals.count(id) == 0)
        collision_intervals.insert({id, {interval}});
    else
        collision_intervals[id].push_back(interval);
    std::sort(collision_intervals[id].begin(), collision_intervals[id].end());
    for(unsigned int i = 0; i + 1 < collision_intervals[id].size(); i++)
        if(collision_intervals[id][i].second + CN_EPSILON > collision_intervals[id][i+1].first)
        {
            collision_intervals[id][i].second = collision_intervals[id][i+1].second;
            collision_intervals[id].erase(collision_intervals[id].begin() + i + 1);
            i--;
        }
}

// 是move添加到 move_constraint中 constraints容器
void SIPP::add_move_constraint(Move move)
{
    std::vector<Move> m_cons(0);
    if(constraints.count({move.id1, move.id2}) == 0)
        constraints.insert({{move.id1, move.id2}, {move}});
    else
    {
        m_cons = constraints.at({move.id1, move.id2});
        bool inserted(false);
        for(unsigned int i = 0; i < m_cons.size(); i++)
        {
            if(inserted)
                break;
            if(m_cons[i].t1 > move.t1)
            {
                if(m_cons[i].t1 < move.t2 + CN_EPSILON)
                {
                    m_cons[i].t1 = move.t1;
                    if(move.t2 + CN_EPSILON > m_cons[i].t2)
                        m_cons[i].t2 = move.t2;
                    inserted = true;
                    if(i != 0)
                        if(m_cons[i-1].t2 + CN_EPSILON > move.t1 && m_cons[i-1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i-1].t2 = move.t2;
                            if(m_cons[i-1].t2 + CN_EPSILON > m_cons[i].t1 && m_cons[i-1].t2 < m_cons[i].t2 + CN_EPSILON)
                            {
                                m_cons[i-1].t2 = m_cons[i].t2;
                                m_cons.erase(m_cons.begin() + i);
                            }
                            inserted = true;
                        }
                }
                else
                {
                    if(i != 0)
                        if(m_cons[i-1].t2 + CN_EPSILON > move.t1 && m_cons[i-1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i-1].t2 = move.t2;
                            inserted = true;
                            break;
                        }
                    m_cons.insert(m_cons.begin() + i, move);
                    inserted = true;
                }
            }
        }
        if(m_cons.back().t2 + CN_EPSILON > move.t1 && m_cons.back().t2 < move.t2 + CN_EPSILON)
            m_cons.back().t2 = move.t2;
        else if(!inserted)
            m_cons.push_back(move);
        constraints.at({move.id1, move.id2}) = m_cons;
    }
}

// 构造算法中需要的约束。
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
                add_move_constraint(Move(con));
        }
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

// 将分段的path链接在一起
Path SIPP::add_part(Path result, Path part)
{
    part.nodes.erase(part.nodes.begin());
    for(auto n: part.nodes)
        result.nodes.push_back(n);
    return result;
}

// 这里实际是sipp搜索的算法，输入是起始的node和可通行的时间间隔，
// 目标node和可通行的时间间隔。每一个node包含一段可通行的时间间隔，
// h_value启发项表，可以直接获取当前点到目标点的启发项，max_f 最大的可通行的时间。
// 主体的算法框架是基于A*算法
/*
- open容器：保存从初始node开始，循环保存下一步的可达的node。同时是按照f值排序的容器。按顺序访问容器中的node，通过visited判断之前是否已经访问过，访问过说明之前有一个更小的f值，可以直接跳过。
- visited 容器：在find_successors函数中向容器中插入元素。
  -  if(it->second.second) 表示的含义：当前的这个node已经作为初始node遍历下一步了，这表明现在没有必要在进行一次了，同时之前的也是f值更小的node
  - 主要的功能：计算从start开始到某个节点时最小的g。同时会计算h，更新f的值
- close容器记录在open容器中pop出来的元素并且不是重复搜索的节点
*/
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
        visited.insert({s.id + s.interval_id * map.get_size(), {s.g, false}});
    }
    Node curNode;
    while(!open.empty())
    {
        curNode = find_min();
        // 当前的节点是否在visited，并且是否检查过。这个也相当于a*启发项的策略？
        auto v = visited.find(curNode.id + curNode.interval_id * map.get_size());
        if(v->second.second)
            continue;
        // 设置当前node检查了
        v->second.second = true;
        // 在 close容器中增加元素
        // std::unordered_map<int, Node> close;
        // parent表示curNode
        auto parent = &close.insert({curNode.id + curNode.interval_id * map.get_size(), curNode}).first->second;
        // 如果当前节点是目标节点
        if(curNode.id == goals[0].id)
        {
           // 遍历目标节点的安全时间间隔维度
            for(unsigned int i = 0; i < goals.size(); i++)
                // 如果当前节点的安全时间间隔和目标的时间间隔有重合
                if(curNode.g - CN_EPSILON < goals[i].interval.second && goals[i].interval.first - CN_EPSILON < curNode.interval.second)
                {
                    // reconstruct_path 返回对应的路径
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
        // find_successors函数
        find_successors(curNode, map, succs, h_values, Node(goals[0].id, 0, 0, goals[0].i, goals[0].j));
        std::list<Node>::iterator it = succs.begin();
        // 遍历找到的有效移动，选择小于最大的时间上限的node,添加到open容器中
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

std::vector<Node> SIPP::get_endpoints(int node_id, double node_i, double node_j, double t1, double t2)
{
    std::vector<Node> nodes;
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
                auto c = collision_intervals[node_id][k];
                bool changed = false;
                if(c.first - CN_EPSILON < n.interval.first && c.second + CN_EPSILON > n.interval.second)
                {
                    nodes.erase(nodes.begin() + i);
                    changed = true;
                }
                else if(c.first - CN_EPSILON < n.interval.first && c.second > n.interval.first)
                {
                    nodes[i].interval.first = c.second;
                    changed = true;
                }
                else if(c.first - CN_EPSILON > n.interval.first && c.second + CN_EPSILON < n.interval.second)
                {
                    nodes[i].interval.second = c.first;
                    nodes.insert(nodes.begin() + i + 1, Node(node_id, 0, 0, node_i, node_j, nullptr, c.second, n.interval.second));
                    changed = true;
                }
                else if(c.first < n.interval.second && c.second + CN_EPSILON > n.interval.second)
                {
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
    return nodes;
}

// 函数作用：判断从start 到 goal 的g
double SIPP::check_endpoint(Node start, Node goal)
{
    double cost = sqrt(pow(start.i - goal.i, 2) + pow(start.j - goal.j, 2));
    if(start.g + cost < goal.interval.first)
        start.g = goal.interval.first - cost;
    // 如果start.id, goal.id中存在约束
    if(constraints.count({start.id, goal.id}) != 0)
    {
        auto it = constraints.find({start.id, goal.id});
        for(unsigned int i = 0; i < it->second.size(); i++)
            if(start.g + CN_EPSILON > it->second[i].t1 && start.g < it->second[i].t2)
                start.g = it->second[i].t2;
    }
    if(start.g > start.interval.second || start.g + cost > goal.interval.second)
        return CN_INFINITY;
    else
        return start.g + cost;
}

Path SIPP::find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values)
{
    this->clear();
    this->agent = agent;
    // 构造算法中需要的约束,将传入的约束进行分类处理。
    make_constraints(cons);
    std::vector<Node> starts, goals;
    std::vector<Path> parts, results, new_results;
    Path part, result;
    int expanded(0);
    if(!landmarks.empty())
    {
        for(unsigned int i = 0; i <= landmarks.size(); i++)
        {
            if(i == 0)
            {
                // 第一个站点，取最开始的就行
                starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
                // goals 取得是所有的在站点landmarks下的安全时间间隔
                goals = get_endpoints(landmarks[i].id1, map.get_i(landmarks[i].id1), map.get_j(landmarks[i].id1), landmarks[i].t1, landmarks[i].t2);
            }
            else
            {
                starts.clear();
                for(auto p:results)
                    starts.push_back(p.nodes.back());
                // 如果是最后一个站点，也就是goal点
                if(i == landmarks.size())
                    goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY).back()};
                else
                    goals = get_endpoints(landmarks[i].id1, map.get_i(landmarks[i].id1), map.get_j(landmarks[i].id1), landmarks[i].t1, landmarks[i].t2);
            }
            if(goals.empty())
                return Path();
            // 局部的路径，搜索start 到 goal的路径. parts是多条路径，从start 到goal
            parts = find_partial_path(starts, goals, map, h_values, goals.back().interval.second);
            expanded += int(close.size());
            new_results.clear();
            if(i == 0)
                for(unsigned int k = 0; k < parts.size(); k++)
                {
                    if(parts[k].nodes.empty())
                        continue;
                    // new_results 包含寻找到的路径
                    new_results.push_back(parts[k]);
                }
            for(unsigned int k = 0; k < parts.size(); k++)
                for(unsigned int j = 0; j < results.size(); j++)
                {
                    if(parts[k].nodes.empty())
                        continue;
                    // 
                    if(fabs(parts[k].nodes[0].interval.first - results[j].nodes.back().interval.first) < CN_EPSILON && fabs(parts[k].nodes[0].interval.second - results[j].nodes.back().interval.second) < CN_EPSILON)
                    {
                        new_results.push_back(results[j]);
                        new_results.back() = add_part(new_results.back(), parts[k]);
                    }
                }
            // 将符合条件的路径付给results
            results = new_results;
            if(results.empty())
                return Path();
            if(i < landmarks.size())
            {
                starts.clear();
                for(auto p:results)
                    starts.push_back(p.nodes.back());
                // offset landmark两个节点之间的距离
                double offset = sqrt(pow(map.get_i(landmarks[i].id1) - map.get_i(landmarks[i].id2), 2) + pow(map.get_j(landmarks[i].id1) - map.get_j(landmarks[i].id2), 2));
                goals = get_endpoints(landmarks[i].id2, map.get_i(landmarks[i].id2), map.get_j(landmarks[i].id2), landmarks[i].t1 + offset, landmarks[i].t2 + offset);
                if(goals.empty())
                    return Path();
                new_results.clear();
                for(unsigned int k = 0; k < goals.size(); k++)
                {
                    double best_g(CN_INFINITY);
                    int best_start_id = -1;
                    // 得到从start到goal中g最下的时间间隔
                    for(unsigned int j = 0; j < starts.size(); j++)
                    {
                        // 
                        double g = check_endpoint(starts[j], goals[k]);
                        if(g < best_g)
                        {
                            best_start_id = j;
                            best_g = g;
                        }
                    }
                    if(best_start_id >= 0)
                    {
                        // 得到最好的g
                        goals[k].g = best_g;
                        // 判断 goals[k]是否存在等待的约束，存在的话，修改g的值
                        if(collision_intervals[goals[k].id].empty())
                            goals[k].interval.second = CN_INFINITY;
                        else
                        {
                            for(auto c:collision_intervals[goals[k].id])
                                if(goals[k].g < c.first)
                                {
                                    goals[k].interval.second = c.first;
                                    break;
                                }
                        }
                        new_results.push_back(results[best_start_id]);
                        if(goals[k].g - starts[best_start_id].g > offset + CN_EPSILON)
                        {
                            new_results.back().nodes.push_back(new_results.back().nodes.back());
                            new_results.back().nodes.back().g = goals[k].g - offset;
                        }
                        new_results.back().nodes.push_back(goals[k]);
                    }
                }

                results = new_results;
                if(results.empty())
                    return Path();
            }
        }
        result = results[0];
    }
    else
    {
        starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
        goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY).back()};
        parts = find_partial_path(starts, goals, map, h_values);
        expanded = int(close.size());
        if(parts[0].cost < 0)
            return Path();
        result = parts[0];
    }
    // result = check_same_node(result);
    result.nodes.shrink_to_fit();
    result.cost = result.nodes.back().g;
    result.agentID = agent.id;
    result.expanded = expanded;
    return result;
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
                        path = fusion_path(path, std::pair<int, int>(id1, id2));
                        flag = true;
                        break;
                    }
            }
         	if (flag)
               break;
        }
    }
    return path;
}

bool SIPP::can_fusion_node(int id1, int id2, Path path) {
    if (id2 - id1 < 2)
        return false; 
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
            interval.second = colls_it->second[j].first;
            intervals.push_back(interval);
            interval.first = colls_it->second[j].second;
        }
    }
    for (auto interval:intervals) {
        if ((interval.first > g1) && (interval.second>g2))
            return true;
    }
    return false;
}

Path SIPP::fusion_path(Path path, std::pair<int, int> node_index) {
    int first = node_index.first+1;
    int second = node_index.second;
    for (int i = first; i < second; ++i)
        path.nodes.erase(std::begin(path.nodes)+first);
    return path;
}