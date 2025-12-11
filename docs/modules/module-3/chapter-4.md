# Chapter 4: Nav2 for Humanoid Path Planning

## Overview
This chapter covers the Navigation2 (Nav2) stack, NVIDIA's advanced navigation framework for humanoid robots. Building upon the ROS 2 foundations from Module 1 and connecting with the perception systems from Module 2 and Isaac ROS from Module 3, students will learn to configure and deploy Nav2 for complex path planning, obstacle avoidance, and autonomous navigation. The chapter emphasizes humanoid-specific navigation challenges and solutions, including dynamic obstacle avoidance and human-aware navigation behaviors that integrate with the perception systems from previous modules.

## Learning Objectives
- Configure Nav2 for humanoid robot navigation systems, building on Module 1's ROS 2 architecture
- Understand advanced path planning algorithms in Nav2 and their integration with perception systems from Modules 2 and 3
- Implement dynamic obstacle avoidance for humanoid robots using Isaac ROS perception inputs
- Apply human-aware navigation techniques that integrate with sensor fusion from Module 2
- Use behavior trees for navigation decision-making with Isaac ROS perception data
- Perform localization in complex environments using Isaac ROS VIO and Module 2's sensor fusion concepts
- Connect Nav2 with Isaac Sim for navigation validation and testing

## Key Concepts

### Navigation2 Architecture
Navigation2 is a comprehensive navigation stack that provides path planning, motion control, and obstacle avoidance capabilities for mobile robots. The architecture consists of several key components: the lifecycle manager, behavior tree executor, controller server, planner server, and recovery server. Each component operates as a lifecycle node, enabling dynamic reconfiguration and robust state management for navigation tasks. This architecture extends the ROS 2 communication patterns from Module 1 and integrates with the perception systems from Module 3's Isaac ROS.

### Path Planning Algorithms
Nav2 implements multiple path planning algorithms optimized for different scenarios. The global planner computes optimal paths from start to goal positions, while the local planner handles real-time obstacle avoidance and trajectory execution. Advanced algorithms include A*, Dijkstra, and custom implementations like the NavFn and Global Planner. The local planner uses algorithms like TEB (Timed Elastic Band) and DWA (Dynamic Window Approach) for dynamic obstacle avoidance, connecting with the obstacle detection capabilities from Isaac ROS in Module 3 and sensor fusion from Module 2.

### Humanoid Navigation Behaviors
Humanoid robot navigation requires special considerations due to the robot's size, dynamics, and interaction with humans. Nav2 supports human-aware navigation through costmap layers that consider human presence and behavior from Isaac ROS perception data. The system can implement social navigation patterns, maintain appropriate distances from humans, and adapt to human movement patterns for safe and efficient navigation, building on the perception concepts from Modules 2 and 3.

## Technical Deep Dive
Navigation2 represents the next generation of ROS navigation, designed specifically for ROS 2 with improved architecture, performance, and capabilities. The system is built around a behavior tree-based execution model that provides flexible and robust navigation behaviors. Building upon the ROS 2 foundations from Module 1, Nav2 integrates seamlessly with the Isaac ROS perception systems from Module 3 and the sensor fusion concepts from Module 2.

The core Nav2 architecture includes:

1. **Lifecycle Manager**: Manages the state transitions of all navigation components, ensuring proper initialization, activation, and cleanup of resources. This component coordinates the startup and shutdown sequences of the navigation stack, following the ROS 2 lifecycle patterns from Module 1.

2. **Behavior Tree Executor**: Executes navigation tasks through a behavior tree framework. This allows for complex decision-making logic and the ability to switch between different navigation behaviors based on environmental conditions from Isaac ROS perception data.

3. **Controller Server**: Handles trajectory control and follows the global path while avoiding local obstacles detected by Isaac ROS perception systems. The controller uses plugins for different control algorithms, with TEB (Timed Elastic Band) being the most commonly used for its ability to handle complex kinematic constraints of humanoid robots.

4. **Planner Server**: Manages both global and local path planning. The global planner creates a path from start to goal, while the local planner adjusts the path in real-time to avoid obstacles detected by Isaac ROS and Module 2's sensor systems.

5. **Recovery Server**: Implements recovery behaviors when the robot gets stuck or encounters navigation failures. These behaviors include simple rotation, move backward, and other strategies to recover from navigation challenges, with obstacle data from Isaac ROS perception.

For humanoid robots, Nav2 requires special configuration to account for the robot's unique characteristics:

- **Footprint Configuration**: Humanoid robots have complex footprints that change during movement. The costmap configuration must account for the robot's full dimensions including limbs during navigation, building on Module 2's kinematics concepts.

- **Dynamic Obstacle Handling**: Humanoid robots often navigate in human-populated environments, requiring sophisticated dynamic obstacle prediction from Isaac ROS perception and avoidance algorithms.

- **Social Navigation**: Nav2 includes plugins for social navigation that consider human behavior patterns from Isaac ROS perception data and maintain appropriate social distances.

The costmap system in Nav2 is highly configurable with multiple layers that integrate with Isaac ROS perception:
- Static layer for known obstacles from the map
- Obstacle layer for Isaac ROS sensor-based obstacle detection
- Inflation layer for safety margins around obstacles
- Voxel layer for 3D obstacle representation from Isaac ROS stereo and depth data
- Range sensor layer for sonar and IR sensors from Module 2

Navigation algorithms in Nav2 are plugin-based, allowing for easy customization:
- Global planners: A*, Dijkstra, NavFn, Global Planner
- Local planners: TEB, DWA, MPC
- Smoother plugins: BSpline, SimpleSmoother

## Code Examples
```python
# Example: Configuring Nav2 for humanoid robot navigation
# Integrating with Isaac ROS perception from Module 3
# Building on ROS 2 communication patterns from Module 1
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan  # From Module 1/2
from std_msgs.msg import String        # From Module 1
import math

class HumanoidNav2Controller(Node):
    def __init__(self):
        super().__init__('humanoid_nav2_controller')

        # Create action client for navigation using Module 1 patterns
        self.nav_to_pose_client = self.create_client(
            NavigateToPose, 'navigate_to_pose')

        # Wait for Nav2 server to be available
        while not self.nav_to_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for NavigateToPose service...')

        # Publisher for current pose using Module 1 patterns
        self.pose_pub = self.create_publisher(PoseStamped, 'current_pose', 10)

        # Subscribe to Isaac ROS perception data (obstacles, etc.)
        self.obstacle_sub = self.create_subscription(
            LaserScan, '/isaac_ros/laser_scan', self.obstacle_callback, 10)

        # Timer for navigation updates
        self.nav_timer = self.create_timer(1.0, self.navigation_callback)

        # Initialize navigation state
        self.current_obstacles = None

    def obstacle_callback(self, msg):
        """Process obstacle data from Isaac ROS perception"""
        self.current_obstacles = msg
        self.get_logger().info(f'Received obstacle data with {len(msg.ranges)} ranges')

    def send_navigation_goal(self, x, y, theta):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()

        # Set the goal pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        # Send the goal
        self.nav_to_pose_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )

        self.nav_to_pose_future.add_done_callback(self.nav_goal_response_callback)

        self.get_logger().info(f'Navigation goal sent to ({x}, {y}, {theta})')

    def nav_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Navigation progress: {feedback.current_pose.pose.position.x:.2f}, '
            f'{feedback.current_pose.pose.position.y:.2f}'
        )

    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

    def navigation_callback(self):
        """Navigation control loop"""
        # Example navigation logic using Isaac ROS perception data
        if self.current_obstacles:
            # Process obstacle data from Isaac ROS for safe navigation
            min_distance = min([r for r in self.current_obstacles.ranges if r > 0], default=float('inf'))
            if min_distance < 0.5:  # Too close to obstacles
                self.get_logger().info('Obstacle detected, adjusting navigation')

        self.get_logger().info('Navigation control loop')

def main(args=None):
    rclpy.init(args=args)
    nav_controller = HumanoidNav2Controller()

    # Send a sample navigation goal
    nav_controller.send_navigation_goal(5.0, 5.0, 0.0)

    try:
        rclpy.spin(nav_controller)
    except KeyboardInterrupt:
        pass
    finally:
        nav_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Diagrams (Text Descriptions)
```
Nav2 Architecture for Humanoid Robots (Building on Module 1 & 2 & 3):
┌─────────────────────────────────────────────────────────────────┐
│                        NAV2 STACK                               │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌──────────────────┐  ┌─────────────────┐ │
│  │ LIFECYCLE       │  │ BEHAVIOR TREE    │  │ CONTROLLER      │ │
│  │ MANAGER         │  │ EXECUTOR         │  │ SERVER          │ │
│  │ (Module 1)      │  │ (Module 1)       │  │ (Module 1)      │ │
│  │ - State         │  │ - Navigation     │  │ - Trajectory    │ │
│  │   Management    │  │   Behaviors      │  │   Control       │ │
│  │ - Resource      │  │ - Decision       │  │ - Path Following│ │
│  │   Management    │  │   Making         │  │ - Obstacle Avoid│ │
│  └─────────────────┘  └──────────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              ↓
        ┌─────────────────────────────────────────────────────────┐
        │                NAVIGATION SERVERS                       │
        │  ┌─────────────┐ ┌─────────────┐ ┌──────────────────┐   │
        │  │  PLANNER    │ │  RECOVERY   │ │  MAP SERVER      │   │
        │  │  SERVER     │ │  SERVER     │ │  (Module 2)      │   │
        │  │             │ │             │ │                  │   │
        │  │  - Global   │ │  - Recovery │ │  - Static Map    │   │
        │  │    Planner  │ │    Behaviors│ │  - Costmaps      │   │
        │  │  - Local    │ │  (Module 1) │ │  - Localization  │   │
        │  │    Planner  │ │             │ │    Maps          │   │
        │  └─────────────┘ └─────────────┘ └──────────────────┘   │
        └─────────────────────────────────────────────────────────┘
                              ↓
        ┌─────────────────────────────────────────────────────────┐
        │           ISAAC ROS PERCEPTION INTEGRATION              │
        │  ┌─────────────────────────────────────────────────────┐│
        │  │  - Obstacle Detection (Module 3)                   ││
        │  │  - Stereo Depth Data (Module 3)                    ││
        │  │  - Object Detection (Module 3)                     ││
        │  │  - VIO Localization (Module 3)                     ││
        │  │  - Sensor Fusion (Module 2 & 3)                    ││
        │  └─────────────────────────────────────────────────────┘│
        └─────────────────────────────────────────────────────────┘
                              ↓
        ┌─────────────────────────────────────────────────────────┐
        │              HUMANOID-SPECIFIC                          │
        │  ┌─────────────────────────────────────────────────────┐│
        │  │  - Complex Footprint Management                     ││
        │  │  - Human-Aware Navigation                           ││
        │  │  - Social Distance Maintenance                      ││
        │  │  - Dynamic Obstacle Prediction                      ││
        │  │  - Multi-Modal Navigation (Walk, Avoid, Stop)       ││
        │  └─────────────────────────────────────────────────────┘│
        └─────────────────────────────────────────────────────────┘
```

## Common Pitfalls
- **Costmap Configuration**: Incorrect costmap settings can lead to poor navigation performance or failure to avoid obstacles, especially when integrating with Isaac ROS perception data.
- **Footprint Definition**: Humanoid robots have complex footprints that must be accurately defined for safe navigation, building on Module 2's kinematics concepts.
- **Parameter Tuning**: Nav2 has many parameters that require careful tuning for humanoid-specific navigation and Isaac ROS perception integration.
- **Localization Dependencies**: Navigation performance heavily depends on accurate localization from Isaac ROS VIO and Module 2's sensor fusion; poor localization leads to navigation failures.
- **Dynamic Obstacle Handling**: Humanoid robots in human environments require sophisticated dynamic obstacle prediction from Isaac ROS perception systems.
- **Integration Complexity**: Connecting Nav2 with Isaac ROS perception requires understanding of Module 1's communication patterns and QoS settings.

## Checkpoints / Mini-Exercises
1. Install and configure Nav2 for a humanoid robot simulation, building on Module 1's ROS 2 setup
2. Set up costmaps with appropriate inflation parameters using Isaac ROS perception data
3. Configure the TEB local planner for humanoid kinematics with Isaac ROS obstacle detection
4. Implement a simple navigation task with Isaac ROS perception-based obstacle avoidance
5. Test navigation in Isaac Sim environment with dynamic obstacles from Module 2 concepts
6. Evaluate navigation performance metrics and tune parameters for Isaac ROS integration
7. Integrate Nav2 with Isaac ROS perception outputs for comprehensive navigation system

## References
- Nav2 Documentation: https://navigation.ros.org/
- Nav2 Tutorials: https://navigation.ros.org/tutorials/
- Nav2 Configuration Guide: https://navigation.ros.org/configuration/index.html
- Behavior Trees in Robotics: https://arxiv.org/abs/1709.00084
- Humanoid Navigation Best Practices: https://ieeexplore.ieee.org/document/8202226