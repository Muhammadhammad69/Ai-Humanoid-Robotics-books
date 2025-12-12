---
title: "چیپٹر 5: منی پروجیکٹ - مکمل ROS 2 سسٹم"
sidebar_label: "چیپٹر 5: منی پروجیکٹ - مکمل ROS 2 سسٹم"
---

# چیپٹر 5: منی پروجیکٹ - مکمل ROS 2 سسٹم

## جائزہ

یہ چیپٹر ماڈیول 1 میں سیکھے گئے تمام تصورات کو ایک ساتھ لاتا ہے ایک سادہ موبائل روبوٹ کے لیے ایک مکمل ROS 2 سسٹم نافذ کر کے۔ آپ ایک کام کرنے والی روبوٹ ایپلی کیشن تیار کریں گے جس میں سینسر پروسیسنگ، کنٹرول سسٹم، اور نیوی گیشن کی صلاحیتیں شامل ہیں۔ یہ ہاتھ سے کام کرنے والا پروجیکٹ یہ ظاہر کرتا ہے کہ کس طرح ٹاپکس، سروسز، ایکشنز، پیرامیٹر، اور URDF ماڈلز کو ایک مربوط روبوٹک سسٹم میں ضم کیا جاتا ہے جو معنی خیز کام انجام دے سکتا ہے۔

پروجیکٹ میں ایک روبوٹ کو تیار کرنا شامل ہے جو مخصوص اہداف تک پہنچ سکے جبکہ سینسر ڈیٹا کا استعمال کرتے ہوئے رکاوٹوں سے بچ سکے۔ اس کے لیے متعدد نوڈس کو نافذ کرنا ضروری ہے جو مختلف ROS 2 نمونوں کے ذریعے بات چیت کرتے ہیں، ایک حقیقی سیمولیشن ماحول تخلیق کرنا، اور تمام اجزاء کو ایک کام کرنے والے سسٹم میں ضم کرنا شامل ہے۔

## سیکھنے کے اہداف

- تمام ماڈیول 1 تصورات کو ایک مکمل روبوٹک سسٹم میں ضم کرنا
- متعدد مربوط ROS 2 نوڈس کو ڈیزائن اور نافذ کرنا
- URDF اور سیمولیشن انضمام کے ساتھ ایک مکمل روبوٹ ماڈل تیار کرنا
- پائی تھون میں سینسر پروسیسنگ اور کنٹرول الگورتھم نافذ کرنا
- پیچیدہ متعدد نوڈ سسٹم شروع کرنے کے لیے لاؤنچ فائلز کا استعمال کرنا
- ایک مکمل ROS 2 ایپلی کیشن کو ڈیبگ اور توثیق کرنا

## کلیدی تصورات

### سسٹم انضمام

پیچیدہ کام کو حاصل کرنے کے لیے مختلف مواصلت کے نمونوں والے متعدد ROS 2 نوڈس کو کامیابی کے ساتھ جوڑنا ضروری ہے جس کے لیے اجزاء کے درمیان احتیاط سے ڈیزائن اور مناسب انٹرفیس کی ضرورت ہوتی ہے۔

### روبوٹ کے رویے کی معماری

پروجیکٹ ایک تہوار معماری کو ظاہر کرتا ہے جس میں ادراک (سینسر پروسیسنگ)، منصوبہ بندی (نیوی گیشن)، اور کنٹرول (موٹر کمانڈز) کمپوننٹس ہیں جو روبوٹ خود مختاری حاصل کرنے کے لیے ایک ساتھ کام کرتے ہیں۔

### سیمولیشن انضمام

پروجیکٹ میں Gazebo سیمولیشن اور RViz وژولائزیشن کے ساتھ انضمام شامل ہے تاکہ ایک مکمل ترقی اور ٹیسٹنگ کا ماحول تخلیق کیا جا سکے۔

### متعدد نوڈس کی م coordination

متعدد نوڈس کے درمیان مناسب coordination کو پیغام کے وقت، حالت کے انتظام، اور تقسیم شدہ سسٹم میں خامی کے ہینڈلنگ کی سمجھ کی ضرورت ہوتی ہے۔

## تکنیکی گہرائی

### پروجیکٹ معماری

مکمل سسٹم میں متعدد مربوط نوڈس شامل ہیں:

**سینسر پروسیسنگ نوڈ**: خام سینسر ڈیٹا (LIDAR، IMU، اودومیٹری) کو پروسیس کرتا ہے اور دوسرے نوڈس کے استعمال کے لیے پروسیس کردہ معلومات شائع کرتا ہے۔

**نیوی گیشن نوڈ**: راستہ منصوبہ بندی اور رکاوٹوں سے بچاؤ کے الگورتھم نافذ کرتا ہے، روبوٹ کو اہداف کی طرف لے جانے کے لیے رفتار کے کمانڈز شائع کرتا ہے۔

**کنٹرولر نوڈ**: روبوٹ کے ہارڈ ویئر (یا سیمولیشن) کے ساتھ انٹرفیس کرتا ہے تاکہ موٹر کمانڈز انجام دیے جا سکیں اور روبوٹ کی حالت کا انتظام کیا جا سکے۔

**وژولائزیشن نوڈ**: RViz میں ڈسپلے کے لیے مارکر اور دیگر معلومات شائع کرتا ہے۔

### مواصلت کے نمونے کا انضمام

سسٹم متعدد مواصلت کے نمونے استعمال کرتا ہے:

- **ٹاپکس**: سینسر ڈیٹا، اودومیٹری، اور رفتار کے کمانڈز مناسب QoS ترتیبات کے ساتھ استعمال کرتے ہیں
- **سروسز**: ڈائنامک دوبارہ کنفیگریشن اور سسٹم کی حالت کے سوالات کے لیے
- **ایکشنز**: فیڈ بیک کے ساتھ ہدف کے مطابق نیوی گیشن کاموں کے لیے

### سسٹم معماری (متن ڈائیگرام)

```
+-------------------+    +-------------------+    +-------------------+
|   سینسر نوڈ       |    |  نیوی گیشن نوڈ    |    |  کنٹرولر نوڈ      |
|                   |    |                   |    |                   |
| - LIDAR پروسیس کریں|    | - راستے منصوبہ بند|    | - موٹر کمانڈز     |
| - اودومیٹری       |    | - رکاوٹوں سے بچیں |    |   انجام دیں       |
| - سینسر ڈیٹا شائع |    | - cmd_vel شائع کریں|    | - حالت کا انتظام  |
|   کریں            |    | - ایکشن سرور     |    | - ہارڈ ویئر I/F   |
+--------+----------+    +--------+----------+    +----------+--------+
         |                      |                          |
         | sensor_msgs          | geometry_msgs            | geometry_msgs
         | /scan, /odom         | /cmd_vel                 | /cmd_vel
         v                      v                          v
+--------+----------------------+--------------------------+--------+
|                     ROS 2 مڈل ویئر لیئر                      |
|                    (DDS ایمپلیمنٹیشن)                         |
+--------+----------------------+--------------------------+--------+
         |                      |                          |
         |                      |                          |
         v                      v                          v
+-------------------+    +-------------------+    +-------------------+
|   وژولائزیشن      |    |   Gazebo سیمولیشن |    |      RViz         |
|      نوڈ          |    |                   |    |                   |
| - مارکر شائع کریں |    | - روبوٹ فزکس     |    | - روبوٹ ڈسپلے کریں|
| - ڈیبگ معلومات    |    | - سینسر ماڈلز    |    | - راستے دکھائیں   |
| - TF ٹرانسفارم   |    | - ماحول          |    | - سینسر ڈیٹا      |
+-------------------+    +-------------------+    +-------------------+
```

## کوڈ کی مثالیں

### مکمل روبوٹ URDF ماڈل

```xml
<?xml version="1.0"?>
<robot name="project_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- بیس لنک -->
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://project_robot/meshes/base.dae"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- LIDAR ماؤنٹ -->
  <joint name="base_to_laser" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="laser_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- چکر -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left_link"/>
    <origin xyz="0 0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_left_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right_link"/>
    <origin xyz="0 -0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_right_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Gazebo انضمام -->
  <gazebo reference="base_link">
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="wheel_left_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="wheel_right_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Gazebo پلگ انز -->
  <gazebo>
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

  <gazebo reference="laser_link">
    <sensor type="ray" name="head_hokuyo_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
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

</robot>
```

### نیوی گیشن نوڈ نافذ کرنا

```python
#!/usr/bin/env python3
"""
مکمل ROS 2 سسٹم پروجیکٹ کے لیے نیوی گیشن نوڈ
رکاوٹوں سے بچاؤ اور ہدف کی نیوی گیشن نافذ کرتا ہے
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from example_interfaces.action import NavigateToPose
import numpy as np
import math


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # نیوی گیشن کی حالت
        self.current_pose = Point()
        self.target_pose = Point()
        self.is_navigating = False
        self.min_distance = 0.5  # رکاوٹوں سے کم از کم محفوظ فاصلہ

        # شائع کنندہ اور سبسکرائبر
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # نیوی گیشن کے لیے ایکشن سرور
        self.navigation_action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_navigation)

        # نیوی گیشن کنٹرول لوپ کے لیے ٹائمر
        self.nav_timer = self.create_timer(0.1, self.navigation_control)

        self.get_logger().info('نیوی گیشن نوڈ انشائیلائز ہوا')

    def odom_callback(self, msg):
        """اودومیٹری سے موجودہ روبوٹ کا پوز اپ ڈیٹ کریں"""
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        # کوارٹینین سے yaw نکالیں (سادہ کردہ)
        # ایک حقیقی نافذ کاری میں، آپ کوارٹینین کو اویلر میں مناسب طریقے سے تبدیل کریں گے

    def laser_callback(self, msg):
        """رکاوٹ کے پتہ لگانے کے لیے لیزر اسکین ڈیٹا کو پروسیس کریں"""
        # غلط رینج فلٹر کریں
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

        if valid_ranges:
            self.min_distance = min(valid_ranges)

    def calculate_navigation_command(self):
        """ہدف اور رکاوٹوں کی بنیاد پر رفتار کمانڈ کا حساب لگائیں"""
        cmd = Twist()

        if not self.is_navigating:
            return cmd

        # ہدف تک کا فاصلہ کا حساب لگائیں
        dist_to_target = math.sqrt(
            (self.target_pose.x - self.current_pose.x)**2 +
            (self.target_pose.y - self.current_pose.y)**2
        )

        # چیک کریں کہ ہدف کے قریب ہے یا نہیں
        if dist_to_target < 0.2:
            self.is_navigating = False
            self.get_logger().info('ہدف کی پوزیشن پر پہنچ گیا')
            return cmd

        # ہدف کی طرف اینگل کا حساب لگائیں
        angle_to_target = math.atan2(
            self.target_pose.y - self.current_pose.y,
            self.target_pose.x - self.current_pose.x
        )

        # سادہ رکاوٹوں سے بچاؤ
        if self.min_distance < 0.8:
            # رکیں اور رکاوٹ سے بچنے کے لیے موڑیں
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # دائیں طرف موڑیں
        else:
            # ہدف کی طرف جائیں
            cmd.linear.x = min(0.5, dist_to_target)  # فاصلے کے ساتھ رفتار کو اسکیل کریں
            cmd.angular.z = angle_to_target * 1.0  # اینگل کی خامی کے مطابق

        return cmd

    def navigation_control(self):
        """مرکزی نیوی گیشن کنٹرول لوپ"""
        if not self.is_navigating:
            return

        cmd = self.calculate_navigation_command()
        self.cmd_vel_publisher.publish(cmd)

    def execute_navigation(self, goal_handle):
        """نیوی گیشن ایکشن انجام دیں"""
        self.get_logger().info(f'نیوی گیشن انجام دیں: ({goal_handle.request.pose.position.x}, {goal_handle.request.pose.position.y})')

        # ہدف کا پوز سیٹ کریں
        self.target_pose = goal_handle.request.pose.position
        self.is_navigating = True

        # ہدف تک پہنچنے تک نیوی گیشن انجام دیں
        while self.is_navigating and not goal_handle.is_cancel_requested:
            # فیڈ بیک شائع کریں
            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.current_pose.position = self.current_pose
            goal_handle.publish_feedback(feedback_msg)

            # دیگر کال بیکس کو پروسیس کرنے کے لیے چھوٹا تاخیر
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        # چیک کریں کہ منسوخ کیا گیا ہے
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.is_navigating = False
            result = NavigateToPose.Result()
            result.error_code = 1  # منسوخ ہو گیا
            return result

        # نیوی گیشن کامیابی کے ساتھ مکمل ہوا
        goal_handle.succeed()
        self.is_navigating = False
        result = NavigateToPose.Result()
        result.error_code = 0  # کامیابی
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('نیوی گیشن نوڈ یوزر کے ذریعے بند کر دیا گیا')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### مکمل سسٹم کے لیے لاؤنچ فائل

```python
# project_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # لاؤنچ کنفیگریشن متغیرات
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # مکمل سسٹم کے لیے نوڈس
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }],
        arguments=[PathJoinSubstitution([
            FindPackageShare('project_robot'),
            'urdf',
            'project_robot.urdf'
        ])]
    )

    navigation_node = Node(
        package='project_robot',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # RViz نوڈ
    rviz_config = PathJoinSubstitution([
        FindPackageShare('project_robot'),
        'rviz',
        'project_robot.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Gazebo لاؤنچ شامل کریں
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('project_robot'),
                'worlds',
                'simple_room.world'
            ])
        }.items()
    )

    # Gazebo میں روبوٹ اسپون کریں
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'project_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='اگر سچ ہے تو سیمولیشن (Gazebo) کلاک استعمال کریں'
        ),
        gazebo,
        spawn_entity,
        robot_state_publisher,
        navigation_node,
        rviz_node
    ])
```

## عام مسائل

- **وقت کے مسائل**: مختلف رفتار پر چلنے والے مختلف نوڈس کو coordination کے مسائل کا سبب بن سکتے ہیں۔ مناسب QoS ترتیبات استعمال کریں اور پیغام کے ٹائم اسٹیمپس پر غور کریں۔
- **کوآرڈینیٹ فریم کا انتظام**: غلط TF ٹرانسفارم نیوی گیشن کی خامیوں کا سبب بن سکتے ہیں۔ یقینی بنائیں کہ تمام فریم مناسب طریقے سے وضاحت شدہ اور منسلک ہیں۔
- **ریسورس کنٹینشن**: متعدد نوڈس کا مشترکہ وسائل تک بغیر مناسب تال میل کے رسائی کرنا ریس کنڈیشنز کا سبب بن سکتا ہے۔
- **پیرامیٹر ٹیوننگ**: نیوی گیشن پیرامیٹر (رفتار، موڑنے کی شرح، رکاوٹ کے پتہ لگانے کی حدیں) کو مستحکم آپریشن کے لیے احتیاط سے ٹیون کرنا ضروری ہے۔
- **خامی کا سامنا کرنا**: ایک نوڈ میں خامیوں کا مناسب طریقے سے سامنا نہ کرنا پورے سسٹم کو ناکام کر سکتا ہے۔

## چیک پوائنٹس / مینی ایکسائزز

1. مکمل روبوٹ URDF نافذ کریں اور اسے RViz میں ٹیسٹ کریں
2. نیوی گیشن نوڈ تیار کریں اور سیمولیشن میں رکاوٹوں سے بچاؤ کو ٹیسٹ کریں
3. نیوی گیشن نوڈ میں ایک سادہ راستہ منصوبہ بندی الگورتھم شامل کریں
4. Gazebo اور RViz کے ساتھ مکمل سسٹم شروع کرنے والی لاؤنچ فائل تیار کریں
5. نیوی گیشن اہداف بھیجنے اور روبوٹ کے رویے کو دیکھنے کے ذریعے مکمل سسٹم کو ٹیسٹ کریں
6. ہدف کو متحرک طریقے سے سیٹ کرنے کے لیے ایک سادہ UI یا سروس شامل کریں

## حوالہ جات

- [ROS 2 نیوی گیشن اسٹیک](https://navigation.ros.org/)
- [روبوٹ اسٹیٹ پبلشر](http://wiki.ros.org/robot_state_publisher)
- [Gazebo ROS انضمام](http://gazebosim.org/tutorials/?tut=ros2_overview)
- [ROS 2 لاؤنچ سسٹم](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)