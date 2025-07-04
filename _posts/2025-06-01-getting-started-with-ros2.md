---
layout: post
title: "Getting Started with ROS2"
date: 2025-06-04
categories: ros2 beginner
---

Welcome to your first ROS2 tutorial! 🚀

If you're just starting with ROS2, this post will help you get a better understanding and also get it running on your machine.

ROS2 is a powerful open-source framework for building robotics software...

---

## What is ROS2?

ROS2 is a open-source framework for robotics applicaiton. it supports scalability with the projects.

---

## Why ROS2?

- Cross-platform
- Real-time support
- Rich Toolset
- Modular and flexible

---

## Installation

We are going to install ROS2 Humble as it is a much more stable release compared to latest release in Ubuntu 22.04 Jammy.

You can refer to the the ROS2 Official documentation for more details about the installation.
[Official Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 

## 1. Set locale

Check if your system supports UTF-8
```bash
locale
```
If 'LANG' is not set or if values do not end with 'UTF-8', you might need to set it.

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  #to verify 
 ```

## 2. Setup sources

Add ROS2 apt repository to your system

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo apt install /tmp/ros2-apt-source.deb
```
## 3. Install ROS2

```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
```

## 4. Environment Setup

For ROS2 commands to work you need to source the ROS2 estup file in the terminal.

```bash
source /opt/ros/humble/setup.bash
```