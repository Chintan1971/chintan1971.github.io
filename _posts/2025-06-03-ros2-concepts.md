---
layout: post
title: "Knowing core ROS2 concepts"
date: 2025-06-03
categories: ROS2 beginner
---

Before getting directly into ROS2 understanding the fundamental concepts are very crucial for working with them and using the right thing.

## 1. Nodes

A Node in ROS2 is the most fundamental unit of computation. Imagine it to be a specialized worker(program) within your robot's software system.

Each node can be used for a single, specific task. For instance, one node might be responsible to move the wheels of a robot while the other node will do the sensor data processing.

Although nodes run independently, they can communicate with each other to achieve a bigger goal.

Nodes can be written in Python or C++.

Example:
 - camera_node: read data from camera.
 - driver_node: sends commands to motors.


## 2. Topics

Topics are the communication channels through which nodes exchange data in form of messages.

It is a unidirectional stream for continuous data flow. This is ideal for things like sensor readings, robot state updates or regular interval commands.

### working:

A node that wants to share data publishes messages to a specific topic.
The nodes which are interested in that data subscribe to the same topic to be able to receive those messages.

A same topic can have multiple subscribers and publishers.

Example:
 - A camera_node publishes data over the topic /camera/images and the the image processing node subscribes to the same node.

 Publisher node:
 ```bash
 self.publisher_ = self.create_publisher(String, 'chatter', 10)
```
 Subscriber node:
 ```bash
 self.subscription = self.create_subscription(String, 'chatter', self.listener_callback, 10)

 ```

## 3. Messages

Messages are the data structures that nodes sends/receives over a topic.

Messages are defined in .msg files. The structure and data types of a message is defined in the same file. For instance a sample .msg file looks like

```
    int32 x
    int32 y
    string message
```

You can also define a custom .msg file if the standard message type doesn't fit your needs.

## 4. Services

Services works like a request/reply communication pattern.

They are used for synchronous, two way communication(client->server) where on node needs to request a specific action or piece of information from another node and then waits for response. It can be used where the node has to wait for response before proceeding.

The server node advertises a service by a name and defines what it does when received a request.

The client node requests message to the server and waits until it is receives the response.

Service files are dedfined with `.srv` extension.


## 5. Actions

Actions are used for long-running tasks which provide feedback during their execution and can be preempted.

It is used when the task might task significantly more time to complete and we want to monitor its progress or even stop it in between.

### working:

An action client sends the signal(goal) to an action server.

The action server starts executing the node and can also provide feedback about the process.

Once the action is completed the server send a final result to the client.

## 6. Parameters

Parameters are the configuration values for any node. We can fetch the values or even change it while the node is running.

Suppose if you have your robot already spawned and running and you changed your mind about its speed, then you can do it directly from terminal using parameters.

There are different ways to set the parameters:
- From the node's code.
- through the YAML config file.
- using terminal command 'ros2 param set ... '
- set values using other node.

Parameters can be very useful while using a pid controller to continuously tune the constants without having the build the program everytime.

## 7. Launch Files

Launch files are very useful when we have multiple nodes to run. To avoid running nodes in each terminal, a launch file can accumulates all of tehm in a single file.

It is generally written in python format, but XML and YAML formats are also supported.

Once you get along with ROS2 projects, you will use launch files more as you will have more than 2 nodes, with different configurations(could be same).



