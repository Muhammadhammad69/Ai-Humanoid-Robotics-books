---
title: "چیپٹر 5: انضمام کے منصوبے"
sidebar_label: "چیپٹر 5: انضمام کے منصوبے"
---

# چیپٹر 5: انضمام کے منصوبے

## جائزہ

یہ چیپٹر ماڈیول 2 کے تمام تصورات کو جامع انضمام کے منصوبوں میں اکٹھا کرتا ہے۔ آپ مکمل ڈیجیٹل ٹوئن ماحول تیار کریں گے جو جیزبو میں فزکس سیمولیشن کو یونیٹی میں زیادہ معیار کے رینڈرنگ کے ساتھ جوڑتا ہے، جو تمام ROS 2 کے ساتھ ضم ہوتا ہے تاکہ ایک مکمل روبوٹکس سیمولیشن ایکو سسٹم فراہم کیا جا سکے۔ یہ منصوبے سیمولیشن کی تکنیکوں کے عملی اطلاق کو ظاہر کرتے ہیں اور اعلی درجے کے روبوٹکس ترقی ورک فلو کے لیے ایک بنیاد فراہم کرتے ہیں۔

اس چیپٹر میں انضمام کے منصوبے ظاہر کرتے ہیں کہ کیسے حقیقی ڈیجیٹل ٹوئن تیار کیے جائیں جو جسمانی روبوٹکس سسٹم کی درست نمائندگی کرتے ہیں۔ ہاتھوں سے نافذ کاری کے ذریعے، آپ سیکھیں گے کہ کیسے پیچیدہ سیمولیشن ماحول کو مربوط کریں جو متعدد سینسرز، حقیقی فزکس، اور مسحور کن وژولائزیشن کو جوڑتا ہے۔ یہ منصوبے آپ کے اپنے روبوٹکس ایپلی کیشنز کے لیے سیمولیشن ماحول تیار کرنے کے ٹیمپلیٹ کے طور پر کام کرتے ہیں۔

## سیکھنے کے اہداف

- جیزبو فزکس سیمولیشن کو یونیٹی رینڈرنگ کے ساتھ ضم کریں تاکہ مکمل ڈیجیٹل ٹوئن ماحول تیار ہو سکے
- متعدد سینسرز اور ایکچوایٹرز کے ساتھ پیچیدہ روبوٹکس سسٹم نافذ کریں
- ایسے حقیقی سیمولیشن منظر تیار کریں جو متعدد روبوٹ کی صلاحیتوں کی جانچ کرتے ہوں
- ایسے جامع سیمولیشن ورک فلو تیار کریں جو ترقی اور ڈپلائمنٹ کو جوڑتے ہوں
- روبوٹکس الگورتھم کے لیے سیمولیشن سے حقیقت کے منتقلی کی توثیق کریں

## کلیدی تصورات

### ڈیجیٹل ٹوئن معماری

ڈیجیٹل ٹوئن جسمانی سسٹم کے ورچوئل نقل ہیں جو سیمولیشن، تجزیات، اور بہتری کو فعال کرتے ہیں۔ روبوٹکس میں، ڈیجیٹل ٹوئن سیمولیشن اور حقیقت کے درمیان خلا کو پاٹتے ہیں، ڈپلائمنٹ سے پہلے روبوٹکس سسٹم کی محفوظ ٹیسٹنگ اور توثیق کی اجازت دیتے ہیں۔

### متعدد ماحول کا انضمام

کامیاب روبوٹکس سیمولیشن کے لیے فزکس سیمولیشن، وژوئل رینڈرنگ، اور کنٹرول سسٹم کے درمیان بے داغ انضمام کی ضرورت ہوتی ہے۔ اس میں متعدد سیمولیشن ماحول کو مربوط کرنا اور یقینی بنانا شامل ہے کہ مطابقت پذیر وقت اور حالت کا انتظام ہو۔

### سیمولیشن میں سینسر فیوژن

مختلف سیمولیٹڈ سینسرز سے ڈیٹا کو جوڑ کر ماحول کے بارے میں زیادہ درست اور قابل اعتماد سمجھ تخلیق کرنا۔ اس کے لیے مناسب ٹائم سینکرونائزیشن اور مختلف سینسر اقسام کے درمیان کوآرڈینیٹ فریم کی ہم آہنگی کی ضرورت ہوتی ہے۔

### سیمولیشن کی توثیق

سیمولیشن کے نتائج کا موازنہ نظریاتی ماڈلز سے اور جب ممکن ہو تو حقیقی دنیا کے ڈیٹا سے کر کے سیمولیشن ماحول کی درستگی کی توثیق کرنا۔

## تکنیکی گہرائی

### ڈیجیٹل ٹوئن معماری

روبوٹکس کے لیے ایک مکمل ڈیجیٹل ٹوئن معماری میں عام طور پر مندرجہ ذیل شامل ہوتا ہے:

**فزکس لیئر**: جیزبو حقیقی فزکس سیمولیشن فراہم کرتا ہے جس میں درست تصادم کا پتہ لگانا، رابطہ کی قوتیں، اور میٹریل کی خصوصیات شامل ہیں۔

**رینڈرنگ لیئر**: یونیٹی زیادہ معیار کا وژوئل رینڈرنگ فراہم کرتا ہے حقیقی سینسر سیمولیشن اور انسانی بات چیت کے لیے۔

**کنٹرول لیئر**: ROS 2 مواصلت کا ایک بیک بون کا کام کرتا ہے جو سیمولیشن کمپوننٹس کو روبوٹکس سافٹ ویئر کے ساتھ جوڑتا ہے۔

**نگرانی لیئر**: سیمولیشن کی حقیقی وقت کی نگرانی کے لیے وژولائزیشن ٹولز اور ڈیبگنگ انٹرفیس۔

### انضمام کے نمونے

جیب مختلف سیمولیشن کمپوننٹس کو ضم کرتے ہوئے کئی نمونے سامنے آتے ہیں:

**بریج نمونہ**: مختلف کوآرڈینیٹ سسٹم اور پیغام فارمیٹس کے درمیان ترجمہ کرنے کے لیے مخصوص بریج نوڈس استعمال کریں۔

**ہم آہنگی نمونہ**: ٹائم سینکرونائزیشن کے میکنزم نافذ کریں تاکہ سیمولیشن کمپوننٹس کے درمیان مطابقت پذیر حالت یقینی بنائی جا سکے۔

**ماڈولریٹی نمونہ**: سیمولیشن کمپوننٹس کو ماڈولر اور دوبارہ استعمال کے قابل ڈیزائن کریں تاکہ مختلف منصوبوں میں استعمال کیا جا سکے۔

### ڈیجیٹل ٹوئن انضمام معماری (متن ڈائیگرام)

```
+------------------------------------------------------------------+
|                    ڈیجیٹل ٹوئن معماری                          |
|                                                                  |
|  +----------------+    +----------------+    +----------------+  |
|  |  جسمانی        |    |  جیزبو         |    |  یونیٹی        |  |
|  |  روبوٹ         |<-->|  فزکس         |<-->|  رینڈرنگ      |  |
|  |  (حقیقی دنیا)  |    |  سیمولیشن     |    |  (بصری)        |  |
|  +----------------+    +----------------+    +----------------+  |
|         |                       |                       |        |
|         | حقیقی وقت ڈیٹا       | فزکس اور            | بصری
|         | اور کمانڈز            | سینسر ڈیٹا         | فیڈ بیک
|         v                       v                       v        |
|  +----------------+    +----------------+    +----------------+  |
|  |  ROS 2         |<-->|  سیمولیشن     |<-->|  انسانی صارف   |  |
|  |  مواصلت        |    |  بریج        |    |  (VR/AR/HMI)   |  |
|  |  لیئر         |    |               |    |                |  |
|  +----------------+    +----------------+    +----------------+  |
|         |                       |                       |        |
|         | ROS پیغامات          | سینسر فیوژن         | VR/AR
|         | اور سروسز            | اور پروسیسنگ         | انٹرفیس
|         v                       v                       v        |
|  +----------------+    +----------------+    +----------------+  |
|  | روبوٹکس       |    | ادراک اور      |    | تربیت اور      |  |
|  | کنٹرول        |    | AI الگورتھم   |    | وژولائزیشن    |  |
|  | الگورتھم      |    | (CV, ML, وغیرہ)|    | ایپلی کیشنز   |  |
|  +----------------+    +----------------+    +----------------+  |
+------------------------------------------------------------------+
```

## کوڈ کی مثالیں

### جیزبو اور یونیٹی انضمام کے ساتھ مکمل روبوٹ URDF

```xml
<?xml version="1.0"?>
<robot name="integration_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- بیس لنک -->
  <link name="base_link">
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="1.5" ixy="0.0" ixz="0.0" iyy="1.5" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.3" length="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.3" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- جیزبو-مخصوص خصوصیات -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.3</mu1>
    <mu2>0.3</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- بائیں چکر -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left_link"/>
    <origin xyz="0 0.3 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_left_link">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <gazebo reference="wheel_left_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- دائیں چکر -->
  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right_link"/>
    <origin xyz="0 -0.3 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_right_link">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <gazebo reference="wheel_right_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- اپر ایرم -->
  <joint name="arm_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="upper_arm_link">
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <gazebo reference="upper_arm_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <!-- نچلا ایرم -->
  <joint name="arm_elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="lower_arm_link"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="lower_arm_link">
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <gazebo reference="lower_arm_link">
    <material>Gazebo/Green</material>
  </gazebo>

  <!-- سینسرز -->
  <joint name="lidar_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.25 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link"/>

  <joint name="camera_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.28 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="camera_link"/>

  <joint name="imu_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link"/>

  <!-- سینسرز کے لیے جیزبو پلگ انز -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="main_lidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_laser.so">
        <topic_name>scan</topic_name>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor type="camera" name="main_camera">
      <pose>0 0 0 0 0 0</pose>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <topic_name>image_raw</topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor type="imu" name="main_imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
        <topic_name>imu</topic_name>
        <body_name>imu_link</body_name>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- ڈیفرنشل ڈرائیو پلگ ان -->
  <gazebo>
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.6</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

</robot>
```

### پیچیدہ ماحول کے ساتھ جیزبو ورلڈ

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="integration_world">
    <!-- فزکس انجن -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <!-- سورج کی لائٹ -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- زمینی سطح -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- روبوٹ -->
    <include>
      <uri>model://integration_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>

    <!-- پیچیدہ ماحولی اشیاء -->
    <model name="table_1">
      <pose>3 2 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="obstacle_cylinder">
      <pose>-2 -1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.5 1.0 1</ambient>
            <diffuse>0.5 0.5 1.0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- متحرک اشیاء -->
    <model name="moving_box">
      <pose>0 3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>1.0 0.5 0.5 1</ambient>
            <diffuse>1.0 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

### ROS 2 انضمام نوڈ

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import numpy as np
import math
from tf2_ros import TransformBroadcaster
import tf_transformations
import cv2


class IntegrationNode(Node):
    """نیا جو تمام سیمولیشن کمپوننٹس کو ضم کرتا ہے"""

    def __init__(self):
        super().__init__('integration_node')

        # CV برج انشائیلائز کریں
        self.cv_bridge = CvBridge()

        # تمام سینسر کی اقسام کے لیے سبسکرائبر تیار کریں
        self.scan_subscription = self.create_subscription(
            LaserScan, '/integration_robot/scan', self.scan_callback, 10)

        self.image_subscription = self.create_subscription(
            Image, '/integration_robot/image_raw', self.image_callback, 10)

        self.imu_subscription = self.create_subscription(
            Imu, '/integration_robot/imu', self.imu_callback, 10)

        self.odom_subscription = self.create_subscription(
            Odometry, '/integration_robot/odom', self.odom_callback, 10)

        # کنٹرول کے لیے شائع کنندہ
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/integration_robot/cmd_vel', 10)

        # پروسیس کردہ ڈیٹا کے لیے شائع کنندہ
        self.merged_scan_publisher = self.create_publisher(
            LaserScan, '/integration_robot/merged_scan', 10)

        self.fused_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/integration_robot/fused_pose', 10)

        # TF براؤڈکاسٹر
        self.tf_broadcaster = TransformBroadcaster(self)

        # روبوٹ کی حالت کے متغیرات
        self.latest_scan = None
        self.latest_image = None
        self.latest_imu = None
        self.latest_odom = None
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # کنٹرول پیرامیٹر
        self.safe_distance = 0.8  # میٹر
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0

        # کنٹرول لوپ کے لیے ٹائمر بنائیں
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('انضمام نوڈ انشائیلائز ہوا')

    def scan_callback(self, msg):
        """LiDAR اسکین ڈیٹا پروسیس کریں"""
        self.latest_scan = msg
        self.get_logger().debug(f'اسکین موصول ہوئی {len(msg.ranges)} رینجس کے ساتھ')

        # اسکین ڈیٹا پروسیس کریں - رکاوٹس کا پتہ لگائیں
        processed_scan = self.process_scan_data(msg)
        self.merged_scan_publisher.publish(processed_scan)

    def image_callback(self, msg):
        """کیمرہ امیج ڈیٹا پروسیس کریں"""
        self.latest_image = msg
        self.get_logger().debug(f'امیج موصول ہوئی: {msg.width}x{msg.height}')

        try:
            # ROS امیج کو OpenCV فارمیٹ میں تبدیل کریں
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            processed_image = self.process_camera_image(cv_image)

            # ضرورت کے مطابق خصوصیات نکالیں یا اشیاء کا پتہ لگائیں
            features = self.extract_features(processed_image)

        except Exception as e:
            self.get_logger().error(f'امیج پروسیس کرنے میں خامی: {e}')

    def imu_callback(self, msg):
        """IMU ڈیٹا پروسیس کریں"""
        self.latest_imu = msg

        # IMU سے اورینٹیشن نکالیں
        orientation_q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        # کوارٹینین کو اولر اینگلز میں تبدیل کریں
        euler = tf_transformations.euler_from_quaternion(orientation_q)
        roll, pitch, yaw = euler

        self.get_logger().debug(f'IMU اورینٹیشن: ({roll:.3f}, {pitch:.3f}, {yaw:.3f})')

    def odom_callback(self, msg):
        """اودومیٹری ڈیٹا پروسیس کریں"""
        self.latest_odom = msg

        # اودومیٹری سے روبوٹ کی پوز اپ ڈیٹ کریں
        pose = msg.pose.pose
        self.robot_pose[0] = pose.position.x
        self.robot_pose[1] = pose.position.y

        # اورینٹیشن کو ہیڈنگ میں تبدیل کریں
        orientation_q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]
        euler = tf_transformations.euler_from_quaternion(orientation_q)
        self.robot_pose[2] = euler[2]  # yaw

        # رفتار نکالیں
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def control_loop(self):
        """مرکزی کنٹرول لوپ جو تمام سینسر ڈیٹا کو ضم کرتا ہے"""
        cmd = Twist()

        if self.latest_scan is not None:
            # LiDAR ڈیٹا استعمال کرتے ہوئے رکاوٹس کی جانچ کریں
            obstacle_info = self.detect_obstacles(self.latest_scan)

            if obstacle_info['closest_distance'] < self.safe_distance:
                # رکاوٹ کا پتہ چلا، رکیں اور موڑیں
                cmd.linear.x = 0.0
                cmd.angular.z = self.max_angular_speed if obstacle_info['direction'] > 0 else -self.max_angular_speed
                self.get_logger().warn(f'رکاوٹ کا پتہ چلا {obstacle_info["closest_distance"]:.2f}م پر، موڑ رہا ہے')
            else:
                # راستہ صاف ہے، آگے بڑھیں
                cmd.linear.x = min(self.max_linear_speed, obstacle_info['closest_distance'] * 0.5)
                cmd.angular.z = 0.0
        else:
            # کوئی اسکین ڈیٹا نہیں ہے، روبوٹ کو روکیں
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # کمانڈ شائع کریں
        self.cmd_vel_publisher.publish(cmd)

        # فیوژن پوز ایسٹیمیٹ شائع کریں
        self.publish_fused_pose()

    def process_scan_data(self, scan_msg):
        """LiDAR ڈیٹا کو شور اور غلط ریڈنگس کو ہٹانے کے لیے پروسیس کریں"""
        processed_msg = LaserScan()
        processed_msg.header = scan_msg.header
        processed_msg.angle_min = scan_msg.angle_min
        processed_msg.angle_max = scan_msg.angle_max
        processed_msg.angle_increment = scan_msg.angle_increment
        processed_msg.time_increment = scan_msg.time_increment
        processed_msg.scan_time = scan_msg.scan_time
        processed_msg.range_min = scan_msg.range_min
        processed_msg.range_max = scan_msg.range_max

        # سادہ فلٹرنگ لاگو کریں
        filtered_ranges = []
        for r in scan_msg.ranges:
            if scan_msg.range_min <= r <= scan_msg.range_max:
                filtered_ranges.append(r)
            else:
                filtered_ranges.append(float('inf'))

        processed_msg.ranges = filtered_ranges
        return processed_msg

    def detect_obstacles(self, scan_msg):
        """روبوٹ کے سامنے رکاوٹس کا پتہ لگائیں"""
        # اسکین کے سامنے 90 ڈگری کا تجزیہ کریں
        center_idx = len(scan_msg.ranges) // 2
        front_range = 45  # ہر طرف کے لیے رے کی تعداد
        front_rays = scan_msg.ranges[center_idx - front_range:center_idx + front_range]

        # درست رینج فلٹر کریں
        valid_ranges = [r for r in front_rays if scan_msg.range_min <= r <= scan_msg.range_max]

        if not valid_ranges:
            return {'closest_distance': float('inf'), 'direction': 0}

        closest_distance = min(valid_ranges)
        closest_idx = front_rays.index(closest_distance) if closest_distance in front_rays else center_idx

        # قریب ترین رکاوٹ کی سمت کا تعین کریں (مثبت = دائیں، منفی = بائیں)
        direction = (closest_idx - front_range) / front_range

        return {'closest_distance': closest_distance, 'direction': direction}

    def process_camera_image(self, cv_image):
        """خصوصیات نکالنے کے لیے کیمرہ امیج پروسیس کریں"""
        # بنیادی امیج پروسیسنگ لاگو کریں
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        return edges

    def extract_features(self, processed_image):
        """پروسیس کردہ امیج سے خصوصیات نکالیں"""
        # مثال: کنٹورز تلاش کریں
        contours, hierarchy = cv2.findContours(
            processed_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        features = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # چھوٹے کنٹورز کو فلٹر کریں
                # باؤنڈنگ مستطیل کا حساب لگائیں
                x, y, w, h = cv2.boundingRect(contour)
                features.append({'x': x, 'y': y, 'width': w, 'height': h})

        return features

    def publish_fused_pose(self):
        """متعدد سینسرز کو جوڑ کر فیوژن پوز ایسٹیمیٹ شائع کریں"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # اودومیٹری سے پوزیشن سیٹ کریں
        pose_msg.pose.pose.position.x = self.robot_pose[0]
        pose_msg.pose.pose.position.y = self.robot_pose[1]
        pose_msg.pose.pose.position.z = 0.0

        # اورینٹیشن سیٹ کریں
        quat = tf_transformations.quaternion_from_euler(0, 0, self.robot_pose[2])
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # کوواریئنس سیٹ کریں (پوزیشن اور اورینٹیشن کے لیے قطری عناصر)
        pose_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # Position x
                                   0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # Position y
                                   0.0, 0.0, 999999, 0.0, 0.0, 0.0,  # Position z
                                   0.0, 0.0, 0.0, 999999, 0.0, 0.0,  # Rotation x
                                   0.0, 0.0, 0.0, 0.0, 999999, 0.0,  # Rotation y
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.05]  # Rotation z

        self.fused_pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('انضمام نوڈ بند ہو گیا')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### یونیٹی انضمام اسکرپٹ

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using System.Collections.Generic;

public class UnityIntegrationController : MonoBehaviour
{
    [Header("روبوٹ کمپوننٹس")]
    public GameObject baseLink;
    public GameObject leftWheel;
    public GameObject rightWheel;
    public GameObject upperArm;
    public GameObject lowerArm;
    public GameObject lidarSensor;
    public GameObject cameraSensor;

    [Header("ROS ترتیبات")]
    public string robotNamespace = "/integration_robot";
    public float publishRate = 50.0f;  // Hz

    private ROSTCPConnector ros;
    private float lastPublishTime = 0.0f;

    // ROS پیغام شائع کنندہ اور سبسکرائبر
    private MessageSubscriber<OdometryMsg> odomSubscriber;
    private MessageSubscriber<TwistMsg> cmdVelSubscriber;
    private Publisher<JointStateMsg> jointStatePublisher;
    private Publisher<OdomMsg> odomPublisher;

    void Start()
    {
        ros = ROSTCPConnector.instance;

        // شائع کنندہ تیار کریں
        jointStatePublisher = ros.AcquirePublisher<JointStateMsg>($"{robotNamespace}/unity_joint_states");
        odomPublisher = ros.AcquirePublisher<OdomMsg>($"{robotNamespace}/unity_odom");

        // روبوٹ کمانڈز کو سبسکرائب کریں
        cmdVelSubscriber = ros.Subscribe<TwistMsg>($"{robotNamespace}/cmd_vel", OnVelocityCommandReceived);
        odomSubscriber = ros.Subscribe<OdometryMsg>($"{robotNamespace}/odom", OnOdomReceived);

        Debug.Log($"یونیٹی انضمام کنٹرولر انشائیلائز ہوا نیم سپیس کے لیے: {robotNamespace}");
    }

    void Update()
    {
        // مخصوص شرح پر جوائنٹ اسٹیٹس شائع کریں
        if (Time.time - lastPublishTime >= 1.0f / publishRate)
        {
            PublishJointStates();
            PublishOdom();
            lastPublishTime = Time.time;
        }

        // موجودہ روبوٹ کی حالت کے مطابق وژولائزیشن اپ ڈیٹ کریں
        UpdateRobotVisualization();
    }

    void OnVelocityCommandReceived(TwistMsg cmd)
    {
        // کمانڈز کے مطابق روبوٹ وژولائزیشن اپ ڈیٹ کریں
        // یہ تصور کرتا ہے کہ روبوٹ کمانڈز کے جواب میں کیسے حرکت کرے گا
        float linearX = (float)cmd.linear.x;
        float angularZ = (float)cmd.angular.z;

        // ڈیفرنشل ڈرائیو کنیمیٹکس کی بنیاد پر چکروں کا گھومنا تصور کریں
        if (leftWheel != null && rightWheel != null)
        {
            // ڈیفرنشل ڈرائیو کنیمیٹکس کی بنیاد پر چکروں کا گھومنا حساب لگائیں
            float leftSpeed = linearX - angularZ * 0.3f; // 0.3m چکر کا فاصلہ
            float rightSpeed = linearX + angularZ * 0.3f;

            leftWheel.transform.Rotate(Vector3.right, leftSpeed * 100 * Time.deltaTime);
            rightWheel.transform.Rotate(Vector3.right, rightSpeed * 100 * Time.deltaTime);
        }

        // بیس لنک کی پوزیشن اور گھماؤ اپ ڈیٹ کریں
        if (baseLink != null)
        {
            baseLink.transform.Translate(Vector3.forward * linearX * Time.deltaTime);
            baseLink.transform.Rotate(Vector3.up, angularZ * Mathf.Rad2Deg * Time.deltaTime);
        }
    }

    void OnOdomReceived(OdometryMsg odomMsg)
    {
        // اودومیٹری کی بنیاد پر روبوٹ کی پوزیشن اور اورینٹیشن اپ ڈیٹ کریں
        if (baseLink != null)
        {
            baseLink.transform.position = new Vector3(
                (float)odomMsg.pose.pose.position.x,
                (float)odomMsg.pose.pose.position.y,
                (float)odomMsg.pose.pose.position.z
            );

            baseLink.transform.rotation = new Quaternion(
                (float)odomMsg.pose.pose.orientation.x,
                (float)odomMsg.pose.pose.orientation.y,
                (float)odomMsg.pose.pose.orientation.z,
                (float)odomMsg.pose.pose.orientation.w
            );
        }
    }

    void UpdateRobotVisualization()
    {
        // موجودہ حالت کی بنیاد پر تمام روبوٹ کمپوننٹس اپ ڈیٹ کریں
        if (upperArm != null)
        {
            // مثال: ایرم حرکت اینیمیٹ کریں
            upperArm.transform.Rotate(Vector3.up, Mathf.Sin(Time.time) * 10 * Time.deltaTime, Space.Self);
        }

        if (lowerArm != null)
        {
            // مثال: نچلا ایرم حرکت اینیمیٹ کریں
            lowerArm.transform.Rotate(Vector3.forward, Mathf.Cos(Time.time) * 15 * Time.deltaTime, Space.Self);
        }
    }

    void PublishJointStates()
    {
        // جوائنٹ اسٹیٹ پیغام بنائیں اور شائع کریں
        JointStateMsg jointStateMsg = new JointStateMsg();
        jointStateMsg.header = new HeaderMsg();
        jointStateMsg.header.stamp = new TimeStamp(ros.Clock.NowSec, ros.Clock.NowNsec);
        jointStateMsg.header.frame_id = "base_link";

        // جوائنٹ نامز پرپولیٹ کریں
        jointStateMsg.name = new List<string> {
            "wheel_left_joint",
            "wheel_right_joint",
            "arm_shoulder_joint",
            "arm_elbow_joint"
        };

        // یونیٹی ٹرانسفارم سے جوائنٹ پوزیشن کا حساب لگائیں
        jointStateMsg.position = new List<double> {
            leftWheel != null ? GetNormalizedWheelRotation(leftWheel) : 0.0,
            rightWheel != null ? GetNormalizedWheelRotation(rightWheel) : 0.0,
            upperArm != null ? upperArm.transform.localRotation.eulerAngles.y * Mathf.Deg2Rad : 0.0,
            lowerArm != null ? lowerArm.transform.localRotation.eulerAngles.z * Mathf.Deg2Rad : 0.0
        };

        // دستیاب ہونے پر رفتار اور کوششیں پرپولیٹ کریں
        jointStateMsg.velocity = new List<double> { 0.0, 0.0, 0.0, 0.0 };
        jointStateMsg.effort = new List<double> { 0.0, 0.0, 0.0, 0.0 };

        jointStatePublisher.Publish(jointStateMsg);
    }

    void PublishOdom()
    {
        // یونیٹی ٹرانسفارم کی بنیاد پر اودومیٹری شائع کریں
        OdomMsg odomMsg = new OdomMsg();
        odomMsg.header = new HeaderMsg();
        odomMsg.header.stamp = new TimeStamp(ros.Clock.NowSec, ros.Clock.NowNsec);
        odomMsg.header.frame_id = "odom";
        odomMsg.child_frame_id = "base_link";

        // پوزیشن سیٹ کریں
        if (baseLink != null)
        {
            odomMsg.pose.pose.position = new PointMsg(
                baseLink.transform.position.x,
                baseLink.transform.position.y,
                baseLink.transform.position.z
            );

            odomMsg.pose.pose.orientation = new QuaternionMsg(
                baseLink.transform.rotation.x,
                baseLink.transform.rotation.y,
                baseLink.transform.rotation.z,
                baseLink.transform.rotation.w
            );
        }

        // ایک حقیقی نافذ کاری میں، آپ وقت کے ساتھ پوزیشن کے تبدیلیوں کی بنیاد پر رفتار کا حساب لگائیں گے
        odomMsg.twist.twist.linear = new Vector3Msg(0.0, 0.0, 0.0);
        odomMsg.twist.twist.angular = new Vector3Msg(0.0, 0.0, 0.0);

        odomPublisher.Publish(odomMsg);
    }

    double GetNormalizedWheelRotation(GameObject wheel)
    {
        // چکر کے گھومنے کو ایک مناسب رینج میں نارملائز کریں
        if (wheel == null) return 0.0;

        // تصور کریں کہ چکر X محور کے ارد گرد گھومتا ہے
        float rotation = wheel.transform.localRotation.eulerAngles.x;

        // ریڈینز میں تبدیل کریں اور نارملائز کریں
        return (rotation % 360) * Mathf.Deg2Rad;
    }
}
```

## عام مسائل

- **ہم آہنگی کے مسائل**: جیزبو، یونیٹی، اور ROS 2 کے درمیان مختلف اپ ڈیٹ شرحیں ہم آہنگی کے مسائل کا سبب بن سکتی ہیں
- **کوآرڈینیٹ سسٹم کی عدم مطابقت**: مختلف سیمولیشن کمپوننٹس کے درمیان غیر مطابق کوآرڈینیٹ سسٹم
- **ریسورس مینجمنٹ**: پیچیدہ سیمولیشنز نمایاں کمپیو ٹیشنل وسائل کا استعمال کر سکتی ہیں
- **انضمام کی پیچیدگی**: متعدد سیمولیشن ٹولز کو جوڑنا دیکھ بھال اور ٹیسٹنگ کی ضرورت رکھتا ہے
- **ٹائم کے مسائل**: اجزاء کے درمیان پیغام کی تاخیریں حقیقی وقت کی کارکردگی کو متاثر کر سکتی ہیں

## چیک پوائنٹس / مینی ایکسائزز

1. تمام سینسرز کے ساتھ مکمل روبوٹ ماڈل تیار کریں اور جیزبو میں ٹیسٹ کریں
2. یونیٹی وژولائزیشن نافذ کریں اور جیزبو کے ساتھ ہم آہنگی کی تصدیق کریں
3. متعدد رکاوٹوں اور متحرک اشیاء کے ساتھ ایک پیچیدہ ماحول تیار کریں
4. نیوی گیشن کے لیے ROS 2 نوڈ میں سینسر ڈیٹا فیوژن ضم کریں
5. مربوط ماحول میں نیوی گیشن کی ٹیسٹنگ کر کے مکمل ڈیجیٹل ٹوئن کی توثیق کریں

## حوالہ جات

- [ROS 2 انضمام ٹیوٹوریلز](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulation.html)
- [جیزبو-یونیٹی انضمام](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/gazebo_integration.md)
- [ڈیجیٹل ٹوئن معماری کے نمونے](https://ieeexplore.ieee.org/document/9143508)
- [روبوٹکس میں متعدد فزکس سیمولیشن](https://www.sciencedirect.com/science/article/pii/S0921889020303567)
- [سیمولیٹڈ ماحول میں سینسر فیوژن](https://arxiv.org/abs/2007.13099)