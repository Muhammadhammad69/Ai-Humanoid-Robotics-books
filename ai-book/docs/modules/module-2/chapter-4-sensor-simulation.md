# Chapter 4: Sensor Simulation

## Overview
This chapter focuses on simulating various sensors crucial for humanoid robotics, including LiDAR, depth cameras, and IMUs (Inertial Measurement Units). Sensor simulation is a critical component of digital twins, allowing developers to test perception algorithms, navigation systems, and control strategies in a safe, repeatable environment. We will explore how to configure and integrate these virtual sensors within Gazebo and Unity environments, ensuring that the simulated data closely matches real-world sensor characteristics and noise patterns.

## Learning Objectives
- Understand the principles and applications of LiDAR simulation in robotics
- Configure depth camera sensors with realistic parameters in simulation environments
- Implement IMU simulation for accurate pose and motion estimation
- Integrate simulated sensors with ROS 2 message formats and topics
- Validate sensor data quality and accuracy for AI perception training

## Key Concepts

### LiDAR Simulation
LiDAR (Light Detection and Ranging) sensors are essential for humanoid robots for navigation, mapping, and obstacle detection. In simulation, LiDAR sensors emit laser beams in a conical pattern and measure the time it takes for each beam to return after reflecting off surfaces. The simulated data closely replicates real-world LiDAR characteristics including beam divergence, range limitations, and noise patterns.

### Depth Camera Simulation
Depth cameras provide 3D information about the environment by measuring the distance to objects in the scene. In simulation, these sensors combine RGB (color) and depth data, often using stereo vision or structured light principles. They are crucial for humanoid robots to understand spatial relationships and perform tasks requiring fine manipulation.

### IMU Simulation
Inertial Measurement Units (IMUs) measure linear acceleration and angular velocity, providing critical information about a robot's motion and orientation. Simulated IMUs include realistic noise models and drift characteristics that match their physical counterparts, enabling accurate state estimation and control algorithms.

### Sensor Fusion in Simulation
Sensor fusion combines data from multiple sensors to create a more accurate and reliable understanding of the environment. In simulation, this allows for testing complex perception systems that integrate LiDAR, cameras, IMUs, and other sensors to achieve robust humanoid robot behavior.

## Technical Deep Dive

### LiDAR Simulation in Gazebo
Gazebo provides realistic LiDAR simulation through plugins that model the physical properties of laser range finders. The most common implementation uses the `libgazebo_ros_laser.so` plugin which publishes sensor_msgs/LaserScan messages compatible with ROS 2.

**Example LiDAR Sensor Configuration in URDF**:
```xml
  <gazebo reference="laser_link">
    <sensor type="ray" name="head_hokuyo_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_head_hokuyo_sensor" filename="libgazebo_ros_laser.so">
        <topic_name>scan</topic_name>
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

### Depth Camera Simulation
Depth cameras in Gazebo are typically implemented using the `libgazebo_ros_openni_kinect.so` plugin, which simulates RGB-D sensors like the Microsoft Kinect. This plugin publishes synchronized image, depth image, and camera info messages.

**Example Depth Camera Configuration in URDF**:
```xml
  <gazebo reference="camera_link">
    <sensor type="depth" name="camera">
      <always_on>true</always_on>
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <format>R8G8B8</format>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <baseline>0.2</baseline>
        <alwaysOn>true</alwaysOn>
        <updateRate>30.0</updateRate>
        <cameraName>camera</cameraName>
        <imageTopicName>rgb/image_raw</imageTopicName>
        <depthImageTopicName>depth/image_raw</depthImageTopicName>
        <pointCloudTopicName>depth/points</pointCloudTopicName>
        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
        <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
        <frameName>camera_depth_optical_frame</frameName>
        <pointCloudCutoff>0.5</pointCloudCutoff>
        <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
        <CxPrime>0.0</CxPrime>
        <Cx>0.0</Cx>
        <Cy>0.0</Cy>
        <focalLength>0.0</focalLength>
        <hackBaseline>0.07</hackBaseline>
      </plugin>
    </sensor>
  </gazebo>
```

### IMU Simulation
IMU sensors in Gazebo are implemented using the `libgazebo_ros_imu.so` plugin, which simulates accelerometers, gyroscopes, and magnetometers. The plugin publishes sensor_msgs/Imu messages with realistic noise characteristics.

**Example IMU Configuration in URDF**:
```xml
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <topicName>imu</topicName>
        <bodyName>imu_link</bodyName>
        <updateRateHZ>100.0</updateRateHZ>
        <gaussianNoise>0.0</gaussianNoise>
        <frameName>imu_link</frameName>
        <initialOrientationAsReference>false</initialOrientationAsReference>
      </plugin>
    </sensor>
  </gazebo>
```

### Sensor Data Processing Node (Python Example)
This ROS 2 node demonstrates how to subscribe to and process simulated sensor data from the digital twin:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np
import cv2

class SensorFusionNode(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Subscribers for different sensor types
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.camera_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10
        )

        self.get_logger().info('Sensor Fusion Node has been started.')

    def lidar_callback(self, msg):
        # Process LiDAR data
        ranges = np.array(msg.ranges)
        # Filter out invalid ranges
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        # Example: Find closest obstacle
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.get_logger().info(f'Closest obstacle: {min_distance:.2f}m')

    def camera_callback(self, msg):
        # Process camera image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Example: Simple edge detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Log image dimensions
            self.get_logger().info(f'Camera image: {cv_image.shape[1]}x{cv_image.shape[0]}')
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def imu_callback(self, msg):
        # Process IMU data
        linear_acc = [msg.linear_acceleration.x,
                     msg.linear_acceleration.y,
                     msg.linear_acceleration.z]
        angular_vel = [msg.angular_velocity.x,
                      msg.angular_velocity.y,
                      msg.angular_velocity.z]

        self.get_logger().info(f'IMU - Linear Acc: [{linear_acc[0]:.2f}, {linear_acc[1]:.2f}, {linear_acc[2]:.2f}]')
        self.get_logger().info(f'IMU - Angular Vel: [{angular_vel[0]:.2f}, {angular_vel[1]:.2f}, {angular_vel[2]:.2f}]')

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()
    rclpy.spin(sensor_fusion_node)
    sensor_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Sensor Simulation Architecture (Text Description)**:
```
+------------------------------------------------------------+
|                    DIGITAL TWIN                            |
|                                                            |
|  +------------------+    +-------------------+    +-------+
|  |   Humanoid       |    |   Sensors         |    | ROS 2 |
|  |   Robot Model    |<--> |   (LiDAR, Cam,    |<-->| Nodes |
|  |                  |    |   IMU, etc.)      |    |       |
|  +------------------+    +-------------------+    +-------+
|         |                        |                        |
|         v                        v                        v
|  +----------------+    +---------------------+    +-------------+
|  | Gazebo Physics |    | Sensor Plugins      |    | Perception  |
|  | Engine         |    | (Laser, Camera,     |    | Algorithms  |
|  |                |    | IMU simulators)     |    |             |
|  +----------------+    +---------------------+    +-------------+
|                                                            |
|  +-----------------------------+--------------------------+
|  | Unity Rendering Engine      |                          |
|  | (Visualizes sensors & data) |                          |
|  +-----------------------------+--------------------------+
+------------------------------------------------------------+
```
This diagram shows how sensors are integrated into the digital twin architecture, with sensor plugins in Gazebo providing data that flows through ROS 2 to perception algorithms.

## Common Pitfalls
- **Sensor Noise Modeling**: Failing to include realistic noise models can lead to overconfident algorithms that perform poorly in the real world.
- **Computational Overhead**: High-resolution sensors (especially depth cameras) can significantly impact simulation performance.
- **Coordinate Frame Mismatches**: Incorrect TF (Transform) trees can cause sensor data to be misinterpreted by perception algorithms.
- **Sensor Fusion Complexity**: Combining data from multiple sensors requires careful synchronization and calibration.

## Checkpoints / Mini-Exercises
1. Add a LiDAR sensor to your humanoid robot model and visualize the scan data in RViz2.
2. Configure a depth camera on your robot and process the point cloud data to detect obstacles.
3. Implement a simple sensor fusion algorithm that combines IMU and visual data to estimate robot orientation.
4. Create a launch file that starts your robot with all three sensor types (LiDAR, camera, IMU) and the sensor processing node.

## References
- [Gazebo Sensors Documentation](https://gazebosim.org/docs/garden/sensors/)
- [ROS 2 Sensor Integration Tutorials](https://docs.ros.org/en/humble/Tutorials/Advanced/URDF/Using-URDF-with-Gazebo.html)
- [Unity Robotics Sensor Simulation](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/sensor_unity_simulation.md)
- [OpenNI Kinect Plugin for Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Camera)