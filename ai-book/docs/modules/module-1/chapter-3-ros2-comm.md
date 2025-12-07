# Chapter 3 - ROS 2 Nodes, Topics, and Services

## Overview

In this chapter, we dive deeper into the fundamental communication mechanisms of ROS 2: nodes, topics, and services. You will learn how to create independent nodes, enable them to exchange real-time data using the publish/subscribe model of topics, and implement request/reply interactions with services. We will also explore Quality of Service (QoS) settings, which are crucial for fine-tuning communication reliability and performance in robotic applications.

---

## Learning Objectives
-   Understand the ROS 2 node lifecycle and how to manage nodes.
-   Implement publisher and subscriber nodes in Python using `rclpy`.
-   Configure Quality of Service (QoS) profiles for topics.
-   Implement service server and client nodes in Python using `rclpy` for request/reply communication.

---

## Node Lifecycle

Unlike ROS 1, ROS 2 introduces a managed node lifecycle, allowing nodes to transition through well-defined states (e.g., `unconfigured`, `inactive`, `active`, `finalized`). This enables better control and predictability, especially in complex and safety-critical systems. For this module, we will primarily focus on nodes in their `active` state.

## Publishers & Subscribers: The Topic Communication Model

Topics are the backbone of real-time, one-to-many communication in ROS 2. They operate on a publish/subscribe model, where publishers send messages to a named topic, and subscribers receive messages from that topic.

### Creating a Simple Publisher

Let's create a Python node that publishes "Hello World" messages to a topic named `/chatter`.

First, ensure you have a ROS 2 package set up. If not, create one:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_ros2_pkg
```

Now, create a Python file `publisher_member_function.py` in `~/ros2_ws/src/my_ros2_pkg/my_ros2_pkg/` (inside the package's Python module directory):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard ROS 2 message type for strings

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # Create a publisher that publishes String messages to the 'chatter' topic
        # The queue_size is 10, meaning it will buffer up to 10 messages if subscribers are slow
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.i = 0
        # Create a timer that calls the timer_callback function every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy library
    minimal_publisher = MinimalPublisher() # Create the node
    rclpy.spin(minimal_publisher) # Keep the node alive until it's manually stopped or ROS shuts down
    minimal_publisher.destroy_node() # Destroy the node explicitly
    rclpy.shutdown() # Shutdown rclpy library

if __name__ == '__main__':
    main()
```

Next, add an entry point in `~/ros2_ws/src/my_ros2_pkg/setup.py` within the `entry_points` dictionary, under `console_scripts`:

```python
    entry_points={
        'console_scripts': [
            'talker = my_ros2_pkg.publisher_member_function:main',
        ],
    },
```

Build your package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_pkg
source install/setup.bash
```

Run your publisher:
```bash
ros2 run my_ros2_pkg talker
```

### Creating a Simple Subscriber

Now, let's create a node that subscribes to the `/chatter` topic and prints received messages. Create `subscriber_member_function.py` in the same directory (`~/ros2_ws/src/my_ros2_pkg/my_ros2_pkg/`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # Create a subscriber that listens for String messages on the 'chatter' topic
        # The queue_size is 10
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Add an entry point in `~/ros2_ws/src/my_ros2_pkg/setup.py`:

```python
    entry_points={
        'console_scripts': [
            'talker = my_ros2_pkg.publisher_member_function:main',
            'listener = my_ros2_pkg.subscriber_member_function:main', # Add this line
        ],
    },
```

Build and source again:
```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_pkg
source install/setup.bash
```

Now, run the talker in one terminal and the listener in another:
```bash
# Terminal 1
ros2 run my_ros2_pkg talker

# Terminal 2
ros2 run my_ros2_pkg listener
```
You should see the listener receiving messages from the talker.

### Quality of Service (QoS)

QoS policies in ROS 2 allow you to configure the trade-offs between reliability, latency, and resource usage for your communication. Key QoS settings include:

*   **Reliability**: `Reliable` (guaranteed delivery) vs. `Best Effort` (data may be lost).
*   **Durability**: `Transient Local` (new subscribers get last published message) vs. `Volatile` (only get messages published after subscription).
*   **History**: `Keep Last` (only store a certain number of messages) vs. `Keep All` (store all messages).
*   **Depth**: Number of messages to keep if `Keep Last` is chosen.

For example, to change the reliability to `Best Effort` for a publisher:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ... inside your Node's __init__ method ...
qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)
```

## Services: Request/Reply Communication

Services are used for synchronous, request-response interactions between nodes.

### Creating a Simple Service Server

Let's create a service server that adds two integers. First, you need to define a custom service message. In `~/ros2_ws/src/my_ros2_pkg/srv/AddTwoInts.srv`:

```
int64 a
int64 b
---
int64 sum
```

Now, update `~/ros2_ws/src/my_ros2_pkg/package.xml` to declare the service:

```xml
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

And `~/ros2_ws/src/my_ros2_pkg/CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)
```

Build your package again:
```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_pkg
source install/setup.bash
```

Now, create the service server `add_two_ints_server.py` in `~/ros2_ws/src/my_ros2_pkg/my_ros2_pkg/`:

```python
import rclpy
from rclpy.node import Node
from my_ros2_pkg.srv import AddTwoInts # Import your custom service type

class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        # Create a service server that listens on the 'add_two_ints' service
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Service Ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request: a: %d b: %d' % (request.a, request.b))
        self.get_logger().info('Sending response: sum: %d' % response.sum)
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_service = AddTwoIntsService()
    rclpy.spin(add_two_ints_service)
    add_two_ints_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Add an entry point in `~/ros2_ws/src/my_ros2_pkg/setup.py`:

```python
    entry_points={
        'console_scripts': [
            'talker = my_ros2_pkg.publisher_member_function:main',
            'listener = my_ros2_pkg.subscriber_member_function:main',
            'add_server = my_ros2_pkg.add_two_ints_server:main', # Add this line
        ],
    },
```

Build and source again. Run the server:
```bash
ros2 run my_ros2_pkg add_server
```

### Creating a Simple Service Client

Now, create the service client `add_two_ints_client.py` in `~/ros2_ws/src/my_ros2_pkg/my_ros2_pkg/`:

```python
import sys
import rclpy
from rclpy.node import Node
from my_ros2_pkg.srv import AddTwoInts # Import your custom service type

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        # Create a service client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait until the service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request() # Create a request object

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req) # Call the service asynchronously

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: ros2 run my_ros2_pkg add_client <int_a> <int_b>')
        return -1

    minimal_client = AddTwoIntsClient()
    minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().error('Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Add an entry point in `~/ros2_ws/src/my_ros2_pkg/setup.py`:

```python
    entry_points={
        'console_scripts': [
            'talker = my_ros2_pkg.publisher_member_function:main',
            'listener = my_ros2_pkg.subscriber_member_function:main',
            'add_server = my_ros2_pkg.add_two_ints_server:main',
            'add_client = my_ros2_pkg.add_two_ints_client:main', # Add this line
        ],
    },
```

Build and source again. In one terminal, run the server. In another, run the client:
```bash
# Terminal 1
ros2 run my_ros2_pkg add_server

# Terminal 2
ros2 run my_ros2_pkg add_client 5 7
```
You should see the client sending a request and the server responding with the sum.

---

## Key Takeaways

*   Nodes are modular executable units in ROS 2.
*   Topics enable asynchronous, one-to-many communication using a publish/subscribe model, ideal for continuous data streams.
*   Services provide synchronous, request/reply communication for single interactions.
*   Quality of Service (QoS) policies allow fine-grained control over communication characteristics like reliability and durability.
*   `rclpy` is the Python client library for interacting with ROS 2.

---

## Exercises

1.  **Modify Publisher Rate**: Change the `timer_period` in the `MinimalPublisher` node to publish messages at a faster rate (e.g., 0.1 seconds) and observe the effect on the subscriber.
2.  **Custom Message Type**: Define a new custom message type (e.g., `Point2D.msg` with `float32 x` and `float32 y`) and create a publisher and subscriber that exchange messages of this type.
3.  **Experiment with QoS**: Modify the `MinimalPublisher` and `MinimalSubscriber` to use `Best Effort` reliability. What happens if you start the subscriber a few seconds after the publisher? Compare this to `Reliable` QoS.
4.  **Custom Service Logic**: Extend the `AddTwoIntsService` to perform a different mathematical operation (e.g., subtraction, multiplication) or a more complex task, and create a corresponding client to test it.
5.  **Multi-Node Setup**: Launch two `talker` nodes and two `listener` nodes. Observe how all listeners receive messages from both talkers (assuming they publish to the same topic). How can you differentiate which talker a message came from?