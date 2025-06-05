---
layout: post
title: "ROS2 examples"
date: 2025-06-02
categories: ros2 beginner
---

ROS2 examples try-on

After you have installed ROS2 and setup the environment, you are ready to run demo nodes,

## 1. C++ talker

Open a new terminal and source the setup file.

```bash
source /opt/ros/humble/setup.bash
```
once it is source you can run the talker file.

```bash
ros2 run demo_nodes_cpp talker
```

In a new terminal tab, source the setup file and run the listener file 

```bash
ros2 run demo_nodes_py listener
```


