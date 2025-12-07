---
title: ROS 2 Overview
description: ROS 2 architecture, middleware, nodes/topics/services/actions overview.
slug: /module-1/ros2-overview
---

# Module 1: ROS 2 Overview

The Robot Operating System (ROS) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors across a wide variety of robotic platforms. ROS 2 is the latest iteration, re-architected to address the limitations of ROS 1, particularly concerning real-time performance, multi-robot systems, and embedded device support.

## ROS 2 Architecture

ROS 2's architecture is built around a distributed, peer-to-peer network of processes called **nodes**. These nodes communicate with each other using a variety of mechanisms, primarily **topics**, **services**, and **actions**.

At its core, ROS 2 replaces ROS 1's custom TCP/IP-based communication system with a **Data Distribution Service (DDS)** implementation. DDS is an open international standard for publish-subscribe communications in real-time systems. This change brings several benefits:

*   **Middleware Agnostic**: ROS 2 can utilize different DDS implementations (e.g., Fast RTPS, Cyclone DDS, OpenSplice), allowing developers to choose the best fit for their needs.
*   **Quality of Service (QoS)**: DDS provides powerful QoS policies (reliability, durability, liveliness, history, etc.) that give developers fine-grained control over how messages are sent and received, crucial for real-time and safety-critical applications.
*   **Improved Security**: DDS includes built-in security features, such as authentication, encryption, and access control.
*   **Multi-robot and Embedded Systems**: DDS's design naturally supports multi-robot communication and is better suited for resource-constrained embedded systems.

## Key Communication Concepts

### Nodes

A **Node** is an executable process that performs computations. In ROS 2, every component of a robot system, from sensor drivers to path planners, is typically implemented as a node.

**Example: Creating a simple ROS 2 Node (Python)**

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node started!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node) # Keep node alive until shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics

**Topics** are the primary means for nodes to exchange data asynchronously. A node can *publish* messages to a topic, and other nodes can *subscribe* to that topic to receive those messages. This is a one-to-many communication pattern.

### Services

**Services** provide a request/response communication pattern. A client node sends a *request* to a service server node, and the server processes the request and sends back a *response*. This is useful for functionalities that require an immediate result, like querying a robot's state or commanding a single action.

### Actions

**Actions** are designed for long-running tasks, providing feedback and the ability to be preempted. Similar to services, they have a goal, result, and feedback mechanism. For example, a "navigate to a goal" action would accept a goal (destination), provide continuous feedback (current position, progress), and return a result (success/failure) upon completion, while also allowing cancellation.

## Middleware

The **middleware** is the layer that enables communication between nodes. As mentioned, ROS 2 uses DDS, providing a standardized and robust communication infrastructure underneath the ROS client libraries (like `rclpy` for Python and `rclcpp` for C++). This abstraction means developers can focus on robot application logic rather than low-level networking details.

## Why ROS 2 for Humanoids?

ROS 2's enhancements, particularly its real-time capabilities, robust QoS, and support for complex multi-component systems, make it an ideal choice for controlling bipedal humanoid robots. Humanoids often require precise, synchronized control of many joints, real-time sensor processing, and complex decision-making, all of which are facilitated by ROS 2's underlying architecture.
