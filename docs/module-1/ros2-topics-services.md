---
title: Topics, Services, and Actions
description: "Robot communication: publish/subscribe, service requests, long-running actions."
slug: /module-1/ros2-topics-services
---

# Topics, Services, and Actions

Effective communication between nodes is crucial for any complex robot system. ROS 2 provides three primary communication mechanisms: Topics, Services, and Actions, each suited for different interaction patterns.

## Topics: Asynchronous Publish-Subscribe

Topics are the most common communication pattern in ROS 2, enabling nodes to send and receive messages asynchronously in a many-to-many fashion. This is ideal for streaming data like sensor readings, joint states, or diagnostic information.

*   **Publisher**: A node that sends messages to a named topic.
*   **Subscriber**: A node that receives messages from a named topic.
*   **Message Type**: All messages published on a topic must conform to a predefined message type (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/Image`).
*   **Quality of Service (QoS)**: Publishers and subscribers can specify QoS policies to control aspects like reliability, history depth, and liveliness, ensuring messages are delivered as required.

### Python Example: Publisher-Subscriber

**Publisher Node (`simple_publisher.py`):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chat_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber Node (`simple_subscriber.py`):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chat_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Synchronous Request-Response

Services are used for operations that require a single request and a single response, typically when the client needs an immediate result from the server. This is a one-to-one, blocking communication pattern.

*   **Service Server**: A node that provides a service, waiting for requests.
*   **Service Client**: A node that sends a request to a service server and waits for a response.
*   **Service Type**: Defines the structure of the request and response messages.

### Python Example: Service Server-Client

**Service Type Definition (`my_robot_interfaces/srv/AddTwoInts.srv`):**

```
int64 a
int64 b
---
int64 sum
```

**Server Node (`add_two_ints_server.py`):**

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AddTwoInts # Custom service type

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Service Server started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}, sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Client Node (`add_two_ints_client.py`):**

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AddTwoInts # Custom service type
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    node.send_request(a, b)

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().error(f'Service call failed: {e}')
            else:
                node.get_logger().info(f'Result of add_two_ints: {response.sum}')
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Long-Running Goal-Feedback-Result

Actions are built on top of topics and services and are designed for long-running, preemptable tasks. They allow a client to send a goal, receive continuous feedback on the goal's progress, and eventually get a result, all while being able to cancel the goal mid-execution.

*   **Action Server**: A node that executes a long-running action.
*   **Action Client**: A node that sends a goal to an action server, monitors its progress, and can cancel it.
*   **Action Type**: Defines the structure of the goal, feedback, and result messages.

### When to Use Which?

*   **Topics**: For continuous, unidirectional, asynchronous data streams (e.g., sensor data, odometry).
*   **Services**: For single, synchronous request/response interactions that are expected to complete quickly (e.g., getting a parameter, triggering a discrete event).
*   **Actions**: For long-running, potentially preemptable tasks that require continuous feedback (e.g., navigating to a point, picking up an object).

Understanding these communication primitives is key to designing robust and scalable ROS 2 robot applications, especially for complex systems like humanoid robots.
