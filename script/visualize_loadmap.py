#!/usr/bin/env python3
import yaml
import matplotlib
# matplotlib.use("Agg")
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

Colors = ['orange']#, 'blue', 'green']


class Animation:
  def __init__(self, map, schedule):
    self.map = map
    self.schedule = schedule

    aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

    self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)

    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()
    # create boundary patch
    xmin = -0.5
    ymin = -0.5
    xmax = map["map"]["dimensions"][0] - 0.5
    ymax = map["map"]["dimensions"][1] - 0.5

    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)

    # creat vetexs and edge
    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
    nodes = []
    for o in map["map"]["nodes"]:
      x, y = o[0], o[1]
      self.patches.append(Circle((x, y), 0.15, facecolor='red', edgecolor='black'))
      nodes.append((x, y))
    for edge in map["map"]["edges"]:
      n0,n1=edge[0], edge[1]
      verts = [
          (nodes[n0][0], nodes[n0][1]),
          (nodes[n1][0], nodes[n1][1])
      ]
      codes = [
         Path.MOVETO,
         Path.LINETO,
      ] 
      path = Path(verts, codes)
      patch = patches.PathPatch(path, lw=2)
      self.patches.append(patch)

    # create agents:
    self.T = 0
    # draw goals first
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      if "goal" in d:
        goals = [d["goal"]]
      if "potentialGoals" in d:
        goals = [goal for goal in d["potentialGoals"]]
      for goal in goals:
        self.patches.append(Rectangle((goal[0] - 0.15, goal[1] - 0.15), 0.3, 0.3, facecolor=Colors[i%len(Colors)], edgecolor='black', alpha=0.5))

   # 在图中画出agent的初始位置 和初始的agents的纹理
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      name = d["name"]
      # self.agents 是一个字典，键是agent的名字，键值是对应的agent的圆形结构
      self.agents[name] = Circle((d["start"][0], d["start"][1]), 0.1, facecolor=Colors[i%len(Colors)], edgecolor='black')
      # original_face_color 设置圆形的颜色，和前面的设置的区别，作为一个属性，后面在reset所有的agent的颜色用到
      self.agents[name].original_face_color = Colors[i%len(Colors)]
      # self.patches 中存储对应初始的agents
      self.patches.append(self.agents[name])
      # 获取最大的cost
      self.T = max(self.T, schedule["schedule"][name][-1]["t"])
      # self.agent_names 中存储是agents的编号，将前缀agent去掉
      self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], name.replace('agent', ''))
      # 设置对齐方式，水平对齐，垂直对齐。居中
      self.agent_names[name].set_horizontalalignment('center')
      self.agent_names[name].set_verticalalignment('center')
      # 把每个agent对应的纹理信息存储在self.artists中
      self.artists.append(self.agent_names[name])

  # 不在for循环之中
  # frames 表示需要更新的时间间隔，基本单位为１
    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int(self.T+1) * 10,
                               interval=100,
                               blit=True)

  def save(self, file_name, speed):
    self.anim.save(
      file_name,
      "ffmpeg",
      fps=10 * speed,
      dpi=200),
      # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

  def show(self):
    plt.show()

  def init_func(self):
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)
    return self.patches + self.artists

# 返回的是 self.patches + self.artist作为动画的输入
  def animate_func(self, i):
    # print('i: {}'.format(i))
    for agent_name in self.schedule["schedule"]:
      agent = schedule["schedule"][agent_name]
      pos = self.getState(i / 10, agent)
      p = (pos[0], pos[1])
      self.agents[agent_name].center = p
      self.agent_names[agent_name].set_position(p)

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
        if np.linalg.norm(pos1 - pos2) < 0.7:
          d1.set_facecolor('red')
          d2.set_facecolor('red')
          print("COLLISION! (agent-agent) ({}, {})".format(i, j))

    return self.patches + self.artists


  def getState(self, t, d):
    idx = 0
    while idx < len(d) and d[idx]["t"] < t:
      idx += 1
    if idx == 0:
      return np.array([float(d[0]["x"]), float(d[0]["y"])])
    elif idx < len(d):
      posLast = np.array([float(d[idx-1]["x"]), float(d[idx-1]["y"])])
      posNext = np.array([float(d[idx]["x"]), float(d[idx]["y"])])
    else:
      return np.array([float(d[-1]["x"]), float(d[-1]["y"])])
    dt = d[idx]["t"] - d[idx-1]["t"]
    t = (t - d[idx-1]["t"]) / dt
    pos = (posNext - posLast) * t + posLast
    return pos



if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("schedule", help="schedule for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
  args = parser.parse_args()


  with open(args.map) as map_file:
    map = yaml.load(map_file)

  with open(args.schedule) as states_file:
    schedule = yaml.load(states_file)

  animation = Animation(map, schedule)

  if args.video:
    animation.save(args.video, args.speed)
  else:
    animation.show()
