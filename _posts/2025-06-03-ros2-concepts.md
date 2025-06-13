---
layout: post
title: "Knowing core ROS2 concepts"
date: 2025-06-03
categories: ROS2 beginner
---

Before getting directly into ROS2 understanding the fundamental concepts are very crucial for working with them and using the right thing.

## 1. Nodes

A Node in ROS2 is a fundamental unit of computation. Imagine it to be a specialized worker(C++ program, Python program) within your robot's software system.

Each node can be used for a single, specific task. For instance, one node might be responsible to move the wheels of a robot while the other node will do the sensor data processing.

Although nodes run independently, they can communicate with each other via topics, services, actions, or parameters.

Example:
 - camera_node: read data from camera.
 - driver_node: sends commands to motors.

#### Below is a simple node(in Python)

 ```bash
 import rclpy
 from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_first_node') ## name your node
        self.get_logger().info('Hello from my node') ## display a message

def main(args=None):
    rclpy.init() ## initialize the ros2 python client  
    node = MyNode() ## create an instance of your node
    rclpy.spin() ## keeps your noded running 
    my_node.destroy_node() ## shutsdown and destroys the node cleanly once done
    rclpy.shutdown() ## shuts down the ros2 client 

if __name__ == "__main__ :
    main()    
 ```


## 2. Topics

Topics are the communication channels through which nodes exchange data in form of messages.

It is a unidirectional stream for continuous data flow. This is ideal for things like sensor readings, robot state updates or regular interval commands.

### working:

A node that wants to share data publishes messages to a specific topic.
The nodes which are interested in that data subscribe to the same topic to be able to receive those messages.

A same topic can have multiple subscribers and publishers.

Example:
 - A camera_node publishes data over the topic /camera/image_raw and the image processing node subscribes to the same topic.

 Publisher node:
 ```bash
 self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
```
 Subscriber node:
 ```bash
 self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
 ```

Note: Here, *Image* refers to the ROS2 messages type which can be known using the command ***ros2 topic list*** and to check the message type for a topic use:
***ros2 topic info /camera/image_raw***


## 3. Messages

Messages are the data structures that nodes sends/receives over a topic. Essentially, the structure of: how you should send your message. 

Messages are defined in .msg files. The structure and data types of a message is defined in the same file. For instance a sample .msg file looks like -

```bash
int32 x
int32 y
string message
```

You can also define a custom .msg file if the standard message type doesn't fit your needs.

To know the full list of supported data-types, check the link:
[Supported data-types](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html#messages)

## 4. Services

Services is an synchronous communication: meaning the client sends a request and then waits for the response/confirmation., it is used in the cases where you want to perform a task only once with a guaranteed reply before going to the next step.

For example: If you want to turn on a camera, you want to make sure that it has turned on before not just hoping that it did, you might want the system to say " camera is turned on."

Other examples might also include calibrating a motor, saving your configuration.

A service consists of: ***Service Server*** and a ***Service Client***

The server node advertises a service by a name, waits for requests and sends back response.

The client node requests message to the server and waits until it is receives the response before continuing.

Service files are defined with `.srv` extension, it consists of both request and response format separated by **---**. 

For Example:
```bash
## adding two numbers
int32 a
int32 b
---
int32 sum
```

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



