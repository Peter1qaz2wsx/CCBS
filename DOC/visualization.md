## 多机调度结果的可视化

- 现在存在两个源码用于使用
  - https://github.com/whoenig/libMultiRobotPlanning　python 代码
    - 优势：可以不用依赖其他的库
    - 
  - http://koupy.net/graphrec.php　基于老版本的QT 
    - 优势：app的功能完备，包含log播放，视频录制等。都集成在一起
    - 劣势：太多功能集成，从代码中获取核心的代码可能麻烦。代码基于老版本的qt.后面使用可能会存在问题。

##  CCBS结果分析

- 每一个section是"move"或者是 "wait".　不存在一个section包含两个动作的、
- 输出的格式是：首先是每一个agent的起始和结束的位置。然后是，每一个agent的运行的过程。