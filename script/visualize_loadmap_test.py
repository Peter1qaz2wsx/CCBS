#!/usr/bin/env python3
import yaml
import matplotlib
# matplotlib.use("Agg")
import xml.dom.minidom as xmldom
from xml.etree import ElementTree as ET
from matplotlib.patches import Circle, Rectangle, Arrow
import matplotlib.patches as patches
from matplotlib.path import Path
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
import argparse
import math

Colors = ['orange', 'black', 'red']
anim_running = True


class Animation:
  def __init__(self, map_inf, task, path_log):
    # map包含输入地图信息的字典。主要包含"node"  "edge"和 "dimensions"
    # 分别表示地图的顶点，边，地图大小的信息
    # task 中包含的是每个 "agent" 的初始的位置和目标的位置。存储的是node的index的信息
    # path_log 包含的是每个agent的路径信息 
    self.map = map_inf
    self.task = task
    self.path_log = path_log
    self.data_i = 0

    aspect = map_inf["dimensions"][0] / map_inf["dimensions"][1]
    self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)

    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()
    self.agent_poses = dict()
    # create boundary patch
    xmin = -2.0
    ymin = -2.0
    xmax = map_inf["dimensions"][0] + 2.0
    ymax = map_inf["dimensions"][1] + 2.0

    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)

    # creat vetexs and edge
    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
    for node in map_inf["nodes"].values():
      x, y = node[0], node[1]
      self.patches.append(Circle((x, y), 0.2, facecolor=Colors[1], edgecolor=Colors[1]))
    for edge in map_inf["edges"].values():
      n0,n1=edge[0], edge[1]
      verts = [
          (map_inf["nodes"][n0][0], map_inf["nodes"][n0][1]),
          (map_inf["nodes"][n1][0], map_inf["nodes"][n1][1])
      ]
      codes = [
         Path.MOVETO,
         Path.LINETO,
      ] 
      path = Path(verts, codes)
      patch = patches.PathPatch(path, lw=0.5)
      self.patches.append(patch)

    # create agents, goals
    goals = []
    i =0
    for name, task_tmp in task.items():
      start_tmp = task_tmp[0]
      goal_tmp = task_tmp[1]
      start_name = "n{}".format(start_tmp)
      goal_name = "n{}".format(goal_tmp)
      start = map_inf["nodes"][start_name]
      goal = map_inf["nodes"][goal_name]
      goals.append(goal)
      goal_tangle =  Circle((goal[0],goal[1]),2.0, facecolor=Colors[0], edgecolor=Colors[1])
      # goal_tangle = Rectangle((goal[0] - 1.5, goal[1] - 1.5), 3, 3, facecolor=Colors[0], edgecolor=Colors[1], alpha=0.5)
      goal_tmp =  self.ax.text(goal[0], goal[1] , name.replace('agent', ''), ha='left', va='center', fontsize=10)
      goal_tmp.set_horizontalalignment('center')
      goal_tmp.set_verticalalignment('center')
      self.patches.append(goal_tangle)
      self.agents[name] = Circle((start[0],start[1]),2.0, facecolor=Colors[2], edgecolor=Colors[1])
      self.agents[name].original_face_color = Colors[2]
      self.patches.append(self.agents[name])
      self.agent_names[name] = self.ax.text(start[0], start[1], name.replace('agent', ''))
      self.agent_names[name].set_horizontalalignment('center')
      self.agent_names[name].set_verticalalignment('center')
      agent_pose = name + ": {}, {}".format(start[0], start[1])
      agent_pose_tex  = self.ax.text(0, i*5 ,agent_pose, ha='left', va='center', fontsize=10)
      i = i+1
      self.agent_poses[name] = agent_pose_tex
      self.artists.append(self.agent_names[name])
      self.artists.append(self.agent_poses[name])
      self.artists.append(goal_tmp)
    t_max = 0
    for value in self.path_log.values():
      time_tmp = value[-1][2]
      if t_max < time_tmp:
        t_max = time_tmp
    # self.init_func()
    print(int((t_max +1)/2) )
    self.fig.canvas.mpl_connect('button_press_event', self.onClick)
    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int((t_max +1)/2) ,
                               interval=100,
                               blit=True)
    # self.anim.event_source.stop()
    # # 修改输入？将当前时间之后的数据清除，增加新的数据。可以实现吗？

    # self.anim.event_source.start()

  def save(self, file_name, speed):
    self.anim.save(
      file_name,
      "ffmpeg",
      fps=10 * speed,
      dpi=200)

  def show(self):
    plt.show()

  def onClick(self, event):
        global anim_running
        ix, iy = event.xdata, event.ydata
        print(ix, iy)
        if anim_running:
          self.anim.event_source.stop()
          # 读取新的地图信息，增加一个“agent”, 删除一条边（在一条边上增加一个永久的agent)
          obstacle_tmp =  Rectangle((ix - 1.5, iy - 1.5), 3, 3, facecolor=Colors[0], edgecolor=Colors[1], alpha=0.5)
          self.patches.append(obstacle_tmp)
          self.init_func()
          # print("self.data_i: {}".format(self.data_i))
          # 删除在i之后的路径，需要结合障碍物的坐标
          self.update_new_path(ix,iy)
          anim_running = False
        else:
          self.anim.event_source.start()
          anim_running = True

  def init_func(self):
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)
    return self.patches + self.artists

  # 更新新的路径，将之前的路径清除，初始的坐标是暂停时的坐标，加上读入新的路径
  def update_new_path(self, ix, iy):
    t = self.data_i
    
    pass
  # 返回的是 self.patches + self.artist作为动画的输入
  def animate_func(self, i):
    self.data_i = i
    # print(i)
    for agent_name in self.path_log.keys():
      agent = self.path_log[agent_name]
      pos = self.getState(i*2, agent)
      p = (pos[0], pos[1])
      # print("pos: {} {}".format(pos[0], pos[1]))
      self.agents[agent_name].center = p
      self.agent_names[agent_name].set_position(p)
      pose_x = round(p[0],3)
      pose_y = round(p[1],3)
      agent_pose = agent_name + ": {}, {}".format(pose_x, pose_y)
      self.agent_poses[agent_name].set_text(agent_pose)
    # reset all colors
    for _,agent in self.agents.items():
      agent.set_facecolor(agent.original_face_color)

    # check drive-drive collisions
    agents_array = [agent for _,agent in self.agents.items()]
    for i in range(0, len(agents_array)):
      for j in range(i+1, len(agents_array)):
        d1 = agents_array[i]
        d2 = agents_array[j]
        pos1 = np.array(d1.center)
        pos2 = np.array(d2.center)
        if np.linalg.norm(pos1 - pos2) < 2.0:
          d1.set_facecolor('red')
          d2.set_facecolor('red')
          print(pos1, pos2)
          print("COLLISION! (agent-agent) ({}, {})".format(i, j))

    return self.patches + self.artists


  def getState(self, t, d):
    idx = 0
    while idx < len(d) and d[idx][2] < t:
      idx += 1
    if idx == 0:
      return np.array([float(d[0][0]), float(d[0][1])])
    elif idx < len(d):
      posLast = np.array([float(d[idx-1][0]), float(d[idx-1][1])])
      posNext = np.array([float(d[idx][0]), float(d[idx][1])])
    else:
      return np.array([float(d[-1][0]), float(d[-1][1])])
    dt = d[idx][2] - d[idx-1][2]
    t = (t - d[idx-1][2]) / dt
    pos = (posNext - posLast) * t + posLast
    return pos

def getMap(map_root, node_map, edge_map ):
  sub_root = map_root[2]
  x_max = 0.0
  y_max = 0.0
  i = 0
  for child in sub_root:
   tag = child.tag.split('}')[-1]
   attrib = child.attrib
   if tag == "node":
    node_id = attrib['id']
    # print(node_id)
    x = float(child[0].text.split(',')[0])
    y = float(child[0].text.split(',')[1])
    # print(x, y)
    node_map[node_id] = (x, y)
    if x > x_max:
     x_max = x
    if y > y_max:
     y_max = y
   if tag == "edge":
    # 将边容器中相同的数据去掉
    if i%2 == 0:
     edge_id = attrib['id']
     source = attrib['source']
     target = attrib['target']
     edge_map[edge_id] = (source, target)
    i = i + 1
  return(x_max, y_max)

def getTaskLog(log_root, task_map, log_map ):
  agent_id = 'agent0'
  id = 0
  for child in log_root:
    tag = child.tag
    if tag == 'agent':
      attrib = child.attrib
      start_id = attrib['start_id']
      goal_id = attrib['goal_id']
      agent_id_str = agent_id.replace('0', str(id))
      id = id + 1
      task_map[agent_id_str] = (start_id, goal_id)
    if tag == 'log':
      for sub_child in child:
        sub_tag = sub_child.tag
        if sub_tag == 'agent':
          start_time = 0.0
          agent_number = sub_child.attrib['number']
          agent_id_str = agent_id.replace('0', str(agent_number))
          log_data = []
          for section in sub_child.iter('section'):
            data = section.attrib
            number = int(data['number'])
            time = float(data['duration'])
            start_time = start_time + time
            if number == 0:
              log_data.append((float(data['start_i']), float(data['start_j']), 0.0))
            log_data.append((float(data['goal_i']), float(data['goal_j']), start_time))
          log_map[agent_id_str] = log_data


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("task_log", help="task and path for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
  args = parser.parse_args()
  map_tree = ET.parse(args.map)
  log_tree = ET.parse(args.task_log)
  map_root = map_tree.getroot()
  log_root = log_tree.getroot()
  node_map = {}
  edge_map = {}
  task_map = {}
  log_map = {}
  x_max, y_max = getMap(map_root, node_map, edge_map)
  getTaskLog(log_root, task_map, log_map )
  map_inf = {}
  map_inf['nodes']  =  node_map
  map_inf['edges'] = edge_map
  map_inf['dimensions'] = (x_max, y_max)
  animation = Animation(map_inf, task_map, log_map)
  if args.video:
    animation.save(args.video, args.speed)
  else:
    animation.show()