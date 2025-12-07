---
title: ROS 2 Nodes
description: Node concepts, creation, lifecycle, and Python integration (rclpy).
slug: /module-1/ros2-nodes
---

# ROS 2 Nodes

In ROS 2, a **node** is a fundamental building block of the robot software system. Each node is an executable program that performs a specific task. Nodes are designed to be modular, allowing for easier development, debugging, and reuse of software components.

## Node Concepts

*   **Modularity**: Each node is responsible for a single, well-defined task (e.g., controlling a motor, processing camera data, planning a path). This enhances code reusability and simplifies system design.
*   **Distribution**: Nodes can run on different machines, in different processes, or within the same process. ROS 2's underlying DDS middleware handles the communication seamlessly.
*   **Communication**: Nodes communicate with each other primarily through topics, services, and actions.
*   **Parameters**: Nodes can have parameters that allow their behavior to be configured dynamically without recompiling the code.

## Creating a ROS 2 Node (Python with `rclpy`)

`rclpy` is the Python client library for ROS 2, providing a clean and intuitive API for creating nodes and interacting with the ROS 2 graph.

### Basic Node Structure

Every `rclpy` node inherits from `rclpy.node.Node`.

```python
# minimal_node.py
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Call the base class constructor with the node name
        super().__init__('my_first_node')
        self.get_logger().info('Hello from my_first_node!')

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS 2 client library
    node = MyNode()       # Create a node instance
    rclpy.spin(node)      # Keep the node alive until Ctrl+C is pressed
    node.destroy_node()   # Cleanly destroy the node
    rclpy.shutdown()      # Shut down the ROS 2 client library

if __name__ == '__main__':
    main()
```

### Building and Running a Node

To make your Python node executable and discoverable by ROS 2, you typically place it within a ROS 2 package.

1.  **Create a ROS 2 package:**
    ```bash
    ros2 pkg create --build-type ament_python my_robot_package
    ```

2.  **Place your script:**
    Save `minimal_node.py` inside `my_robot_package/my_robot_package/`.

3.  **Update `setup.py`:**
    In `my_robot_package/setup.py`, add an entry point under `entry_points`:
    ```python
    # setup.py (partial)
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.minimal_node:main',
        ],
    },
    ```

4.  **Build the package:**
    Navigate to your ROS 2 workspace root and run:
    ```bash
    colcon build --packages-select my_robot_package
    ```

5.  **Source the workspace:**
    ```bash
    source install/setup.bash # or setup.zsh, setup.ps1
    ```

6.  **Run the node:**
    ```bash
    ros2 run my_robot_package my_node
    ```

## Node Lifecycle

ROS 2 introduces a managed lifecycle for nodes, particularly for "lifecycle nodes," which are crucial for deterministic behavior in complex robotic systems. Standard nodes have a simpler lifecycle (created, run, destroyed).

### Standard Node Lifecycle

*   **Initialization**: The node's `__init__` method is called.
*   **Execution (`rclpy.spin`)**: The node enters its main loop, processing callbacks (e.g., from subscriptions, timers).
*   **Shutdown**: The node is destroyed (e.g., `destroy_node()`) and `rclpy` shuts down.

### Managed Lifecycle Nodes (Advanced)

For advanced scenarios, ROS 2 provides `LifecycleNodes` that transition through well-defined states (e.g., `unconfigured`, `inactive`, `active`, `finalized`). Each state transition can have associated callbacks, allowing for robust error handling and resource management, which is vital for industrial and safety-critical applications. This ensures that a robot system can start up, shut down, or recover from failures in a predictable manner.

## Python Integration with `rclpy`

`rclpy` simplifies ROS 2 development in Python by:

*   **Abstracting DDS**: You don't directly interact with DDS; `rclpy` handles it.
*   **Event-driven programming**: ROS 2 operations (publish, subscribe, service calls, etc.) are often handled via callbacks.
*   **Concurrency**: `rclpy` supports both single-threaded and multi-threaded executors for processing callbacks.

Understanding ROS 2 nodes is foundational for building any robotic application. They are the independent computational units that, when combined, form the complete intelligence and control system of a robot.
