---
title: "چیپٹر 4: سینسر سیمولیشن"
sidebar_label: "چیپٹر 4: سینسر سیمولیشن"
---

# چیپٹر 4: سینسر سیمولیشن

## جائزہ

یہ چیپٹر روبوٹکس ادراک اور نیوی گیشن کے لیے اہم سینسرز کی سیمولیشن پر مرکوز ہے۔ آپ سیکھیں گے کہ جیزبو اور یونیٹی دونوں ماحول میں LiDAR، کیمرے، IMU، اور دیگر سینسرز کی حقیقی سیمولیشن کیسے نافذ کریں۔ سینسر سیمولیشن کو سمجھنا ادراک الگورتھم کی ترقی اور ٹیسٹنگ کے لیے اہم ہے، کیونکہ یہ جسمانی ہارڈ ویئر پر ڈپلائے کرنے سے پہلے محفوظ اور قیمت کے لحاظ سے مؤثر توثیق کی اجازت دیتا ہے۔

سینسر سیمولیشن ادراک الگورتھم کی ترقی کو فعال کرتا ہے جس کے لیے مہنگے جسمانی سینسرز تک رسائی یا سامان کو نقصان پہنچانے کے خطرے کی ضرورت نہیں ہوتی۔ حقیقی سینسر سیمولیشن کو ہر سینسر کی قسم کے پیچھے جسمانی اصولوں، ان کی شور کی خصوصیات، اور ماحول کے ساتھ بات چیت کو سمجھنے کی ضرورت ہوتی ہے۔ یہ چیپٹر جیزبو میں فزکس-مبنی سیمولیشن اور یونیٹی میں وژوئل رینڈرنگ کے پہلوؤں کو کور کرتا ہے۔

## سیکھنے کے اہداف

- مختلف سینسر کی اقسام اور ان کی سیمولیشن کے پیچھے اصولوں کو سمجھنا
- مناسب شور اور خامی کی ماڈلنگ کے ساتھ حقیقی LiDAR سیمولیشن نافذ کرنا
- حقیقی آپٹیکل خصوصیات کے ساتھ کیمرہ سیمولیشن تیار کرنا
- مناسب شور اور ڈریفٹ کی خصوصیات کے ساتھ IMU سینسرز کی سیمولیشن کرنا
- ROS 2 پیغام فارمیٹس کے ساتھ سینسر ڈیٹا ضم کرنا
- سینسر سیمولیشن کی درستگی اور کارکردگی کی توثیق کرنا

## کلیدی تصورات

### سینسر فزکس اور ماڈلنگ

ہر سینسر کی قسم مختلف جسمانی اصولوں پر کام کرتی ہے جن کی سیمولیشن میں درست ماڈلنگ کی ضرورت ہوتی ہے۔ ان اصولوں کو سمجھنا حقیقی سینسر کے رویے کو تخلیق کرنے کے لیے ضروری ہے۔

### شور اور خامی کی ماڈلنگ

حقیقی سینسرز میں ذاتی شور، بائس، اور ڈریفٹ ہوتا ہے جسے حقیقی سیمولیشن تخلیق کرنے کے لیے ماڈل کرنا چاہیے۔ مناسب شور ماڈلنگ یہ یقینی بناتی ہے کہ سیمولیشن میں ترقی یافتہ الگورتھم حقیقی ہارڈ ویئر پر اچھا کام کریں گے۔

### سینسر فیوژن

ماحول کے بارے میں زیادہ درست اور قابل اعتماد سمجھ تخلیق کرنے کے لیے متعدد سینسرز سے ڈیٹا کو جوڑنا۔ سینسر فیوژن کی سیمولیشن کو مناسب ٹائم سینکرونائزیشن اور کوآرڈینیٹ فریم کی ہم آہنگی کی ضرورت ہوتی ہے۔

### کارکردگی کے جاتے

سینسر سیمولیشن کمپیو ٹیشنل طور پر کثیر کام ہو سکتا ہے، خاص طور پر کیمرے جیسے زیادہ ریزولوشن والے سینسرز کے لیے۔ حقیقی وقت کی کارکردگی کے لیے کارآمد سیمولیشن کی تکنیکیں ضروری ہیں۔

## تکنیکی گہرائی

### LiDAR سیمولیشن

LiDAR (لائٹ ڈیٹیکشن اینڈ رینجنگ) سینسرز لیزر پلسز کو ایمیٹ کرکے اور روشنی کو واپس آنے میں لگنے والے وقت کو ناپ کر کام کرتے ہیں جو سطحوں سے ٹکرا کر واپس آتی ہے۔ سیمولیشن کو مندرجہ ذیل کا احاطہ کرنا چاہیے:

**رے کاسٹنگ**: ماحول میں اشیاء کے ساتھ ملاقات کا پتہ لگانے والی لیزر بیمز کے اخراج کی سیمولیشن۔

**رینج اور ریزولوشن**: سینسر کی زیادہ سے زیادہ رینج، کم سے کم رینج، زاویہ کا ریزولوشن، اور نظارہ کے میدان کی ماڈلنگ۔

**شور اور خامیاں**: حقیقی LiDAR سینسرز کی ناکاملیوں کی سیمولیشن کرنے والے حقیقی شور ماڈلز کا اضافہ۔

**ماحولی عوامل**: مختلف مواد اور ماحولی حالات کا پیمائش کی درستگی پر اثر ماڈل کرنا۔

### کیمرہ سیمولیشن

کیمرہ سینسرز بصری معلومات کو قبضہ کرتے ہیں اور حقیقی تصاویر کی سیمولیشن کے لیے جامع رینڈرنگ کی ضرورت ہوتی ہے:

**آپٹیکل خصوصیات**: فوکل لمبائی، نظارہ کا میدان، ڈسٹورشن پیرامیٹر، اور دیگر آپٹیکل خصوصیات کی ماڈلنگ۔

**امیج فارمیشن**: 3D دنیا کے کوآرڈینیٹس کو 2D امیج کوآرڈینیٹس پر پروجیکٹ کرنے کے عمل کی سیمولیشن۔

**لائٹنگ ایفیکٹس**: منظر میں ایکسپوزر، ڈائنا مک رینج، اور لائٹنگ کی حالت کا احتساب۔

**ڈسٹورشن**: لینس ڈسٹورشن ایفیکٹس جیسے بیرل اور پن کشن ڈسٹورشن کی ماڈلنگ۔

### IMU سیمولیشن

IMU (انرٹیل میزورمینٹ یونٹ) سینسرز لینیئر ایکسلریشن اور اینگولر رفتار کو ناپتے ہیں:

**شور کی خصوصیات**: حقیقی IMU کی وضاحت کرنے والے سینسر شور، بائس، اور ڈریفٹ کی ماڈلنگ۔

**گریویٹی کمپنیشن**: ناپے گئے اقدار میں گریویٹیشنل ایکسلریشن کا مناسب احتساب۔

**کوآرڈینیٹ سسٹم کی ہم آہنگی**: روبوٹ کے کوآرڈینیٹ سسٹم کے ساتھ مناسب ہم آہنگی کو یقینی بنانا۔

### سینسر سیمولیشن معماری (متن ڈائیگرام)

```
+-----------------------------------------------------------+
|                   سینسر سیمولیشن                         |
|                                                           |
|  +----------------+    +----------------+    +----------+ |
|  |   جسمانی       |    |   سیمولیشن     |    |   ROS 2  | |
|  |   ماحول        |----|   (جیزبو/      |----|   پیغام  | |
|  |                |    |   یونیٹی)      |    |   فارمیٹس| |
|  +----------------+    +----------------+    +----------+ |
|         |                       |                    |    |
|         | ماحولی خصوصیات       | سینسر ماڈلز       | ROS
|         | (مواد، لائٹنگ،       | (فزکس، رینڈرنگ)   | پیغامات
|         | وغیرہ)                |                   | (سینسرز،
|         |                       |                   |  پیرامیٹر)
|         v                       v                    v    |
|  +----------------+    +----------------+    +----------+ |
|  |   رے کاسٹنگ   |    |   امیج         |    |   سینسر  | |
|  |   / کولیژن     |    |   پروسیسنگ    |    |   ڈیٹا  | |
|  |   ڈیٹیکشن     |    |   (کیمرے)      |    |   ٹاپکس | |
|  +----------------+    +----------------+    +----------+ |
|         |                       |                    |    |
|         | رینج ڈیٹا           | امیج ڈیٹا         | معیاری
|         | (LIDAR)              | (کیمرے)            | پیغام
|         |                       |                   | اقسام
|         v                       v                    v    |
|  +----------------+    +----------------+    +----------+ |
|  |   شور ماڈلنگ  |    |   ڈسٹورشن     |    |   جوائنٹ | |
|  |   (IMU, LIDAR) |    |   ماڈلنگ      |    |   اسٹیٹ  | |
|  |                |    |   (کیمرے)      |    |   (اگر   | |
|  +----------------+    +----------------+    |   ضرورت | |
|         |                       |             +----------+ |
|         +-----------------------+--------------------+       |
|                                 |                           |
|                           +-----v-----+                     |
|                           |  فیوژن    |                     |
|                           |  اور      |                     |
|                           |  فلٹرنگ  |                     |
|                           +-----------+                     |
+-----------------------------------------------------------+
```

## کوڈ کی مثالیں

### جیزبو LiDAR سینسر کنفیگریشن

```xml
<sdf version="1.7">
  <model name="lidar_model">
    <link name="lidar_link">
      <pose>0.2 0 0.1 0 0 0</pose>

      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>

    <sensor name="lidar_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <topic>scan</topic>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle> <!-- -90 ڈگری -->
            <max_angle>1.570796</max_angle>   <!-- 90 ڈگری -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <always_on>true</always_on>
      <visualize>true</visualize>

      <!-- حقیقی سیمولیشن کے لیے شور پیرامیٹر -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev> <!-- 1cm معیاری انحراف -->
      </noise>
    </sensor>

    <!-- ROS 2 انضمام کے لیے جیزبو پلگ ان -->
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </model>
</sdf>
```

### جیزبو کیمرہ سینسر کنفیگریشن

```xml
<sdf version="1.7">
  <model name="camera_model">
    <link name="camera_link">
      <pose>0.2 0 0.2 0 0 0</pose>

      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.1 0.1 1</ambient>
          <diffuse>0.8 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>

    <sensor name="camera_sensor" type="camera">
      <pose>0 0 0 0 0 0</pose>
      <topic>camera/image_raw</topic>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 ڈگری -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <always_on>true</always_on>
      <visualize>true</visualize>
    </sensor>

    <!-- ROS 2 کے لیے کیمرہ پلگ ان -->
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/image_raw:=camera/image_raw</remapping>
        <remapping>~/camera_info:=camera/camera_info</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>100.0</max_depth>
    </plugin>
  </model>
</sdf>
```

### جیزبو IMU سینسر کنفیگریشن

```xml
<sdf version="1.7">
  <model name="imu_model">
    <link name="imu_link">
      <pose>0 0 0.3 0 0 0</pose>

      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.000001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000001</iyy>
          <iyz>0</iyz>
          <izz>0.000001</izz>
        </inertia>
      </inertial>
    </link>

    <sensor name="imu_sensor" type="imu">
      <pose>0 0 0 0 0 0</pose>
      <topic>imu</topic>
      <update_rate>100</update_rate>
      <always_on>true</always_on>
      <visualize>false</visualize>

      <!-- IMU شور پیرامیٹر -->
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev> <!-- 1 mrad/s -->
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </z>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
        </angular_velocity>

        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev> <!-- 1.7 mg -->
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </z>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0017</bias_stddev>
        </linear_acceleration>
      </imu>
    </sensor>

    <!-- ROS 2 کے لیے IMU پلگ ان -->
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <topic>imu</topic>
    </plugin>
  </model>
</sdf>
```

### ROS 2 سینسر پروسیسنگ نوڈ

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import math
from tf2_ros import TransformListener, Buffer
import cv2


class SensorProcessorNode(Node):
    """نیا جو سیمولیٹڈ سینسر ڈیٹا کو پروسیس کرتا ہے"""

    def __init__(self):
        super().__init__('sensor_processor_node')

        # امیج پروسیسنگ کے لیے CV برج انشائیلائز کریں
        self.cv_bridge = CvBridge()

        # مختلف سینسر کی اقسام کے لیے سبسکرائبر تیار کریں
        self.scan_subscription = self.create_subscription(
            LaserScan, '/my_robot/scan', self.scan_callback, 10)

        self.image_subscription = self.create_subscription(
            Image, '/my_robot/camera/image_raw', self.image_callback, 10)

        self.imu_subscription = self.create_subscription(
            Imu, '/my_robot/imu', self.imu_callback, 10)

        # پروسیس کردہ ڈیٹا کے لیے شائع کنندہ
        self.processed_scan_publisher = self.create_publisher(
            LaserScan, '/my_robot/processed_scan', 10)

        self.obstacle_detection_publisher = self.create_publisher(
            LaserScan, '/my_robot/obstacle_scan', 10)

        # کوآرڈینیٹ ٹرانسفارمیشن کے لیے TF بفر
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # روبوٹ اسٹیٹ متغیرات
        self.latest_scan = None
        self.latest_image = None
        self.latest_imu = None

        self.get_logger().info('سینسر پروسیسر نوڈ انشائیلائز ہوا')

    def scan_callback(self, msg):
        """LiDAR اسکین ڈیٹا پروسیس کریں"""
        self.latest_scan = msg
        self.get_logger().debug(f'اسکین موصول ہوئی {len(msg.ranges)} رینجس کے ساتھ')

        # اسکین ڈیٹا پروسیس کریں - رکاوٹس کا پتہ لگائیں
        processed_scan = self.process_lidar_data(msg)
        self.processed_scan_publisher.publish(processed_scan)

        # روبوٹ کے سامنے رکاوٹس کا پتہ لگائیں
        front_ranges = msg.ranges[len(msg.ranges)//2-30:len(msg.ranges)//2+30]
        valid_ranges = [r for r in front_ranges if msg.range_min <= r <= msg.range_max]

        if valid_ranges:
            min_distance = min(valid_ranges)
            if min_distance < 1.0:  # 1 میٹر سے کم
                self.get_logger().warn(f'{min_distance:.2f}م پر رکاوٹ کا پتہ چلا')

    def image_callback(self, msg):
        """کیمرہ امیج ڈیٹا پروسیس کریں"""
        self.latest_image = msg
        self.get_logger().debug(f'امیج موصول ہوئی: {msg.width}x{msg.height}')

        try:
            # ROS امیج کو OpenCV فارمیٹ میں تبدیل کریں
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # مثال: رنگ کی فلٹرنگ کا استعمال کرتے ہوئے سادہ اشیاء کا پتہ لگانا
            processed_image = self.process_camera_image(cv_image)

            # ایک حقیقی نافذ کاری میں، آپ پروسیس کردہ امیج شائع کر سکتے ہیں
            # یا مزید پروسیسنگ کے لیے خصوصیات نکال سکتے ہیں

        except Exception as e:
            self.get_logger().error(f'امیج پروسیس کرنے میں خامی: {e}')

    def imu_callback(self, msg):
        """IMU ڈیٹا پروسیس کریں"""
        self.latest_imu = msg

        # اورینٹیشن اور اینگولر رفتار نکالیں
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        self.get_logger().debug(
            f'IMU - اورینٹیشن: ({orientation.x:.3f}, {orientation.y:.3f}, '
            f'{orientation.z:.3f}, {orientation.w:.3f})'
        )

    def process_lidar_data(self, scan_msg):
        """LiDAR ڈیٹا پر پروسیسنگ لاگو کریں"""
        # اسکین پیغام کا کاپی بنائیں
        processed_msg = LaserScan()
        processed_msg.header = scan_msg.header
        processed_msg.angle_min = scan_msg.angle_min
        processed_msg.angle_max = scan_msg.angle_max
        processed_msg.angle_increment = scan_msg.angle_increment
        processed_msg.time_increment = scan_msg.time_increment
        processed_msg.scan_time = scan_msg.scan_time
        processed_msg.range_min = scan_msg.range_min
        processed_msg.range_max = scan_msg.range_max

        # شور اور غلط ریڈنگس کو ہٹانے کے لیے فلٹرنگ لاگو کریں
        filtered_ranges = []
        for r in scan_msg.ranges:
            if scan_msg.range_min <= r <= scan_msg.range_max:
                # سادہ فلٹرنگ لاگو کریں (حقیقی نافذ کاری میں، زیادہ ترقی یافتہ فلٹر)
                filtered_ranges.append(r)
            else:
                # غلط ریڈنگس کے لیے زیادہ سے زیادہ رینج استعمال کریں
                filtered_ranges.append(float('inf'))

        processed_msg.ranges = filtered_ranges
        return processed_msg

    def process_camera_image(self, cv_image):
        """کیمرہ امیج پر پروسیسنگ لاگو کریں"""
        # مثال: رنگ-مبنی اشیاء کا پتہ لگانا
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # لال رنگ کا پتہ لگانے کے لیے رینج کی وضاحت کریں
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # پتہ چلی اشیاء کے کنٹورز تلاش کریں
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # پتہ چلی اشیاء کے گرد باؤنڈنگ باکسز کھینچیں
        result_image = cv_image.copy()
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # چھوٹے ڈیٹیکشنز کو فلٹر کریں
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(result_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        return result_image

    def get_robot_pose_from_tf(self):
        """TF ٹری سے روبوٹ کی پوز حاصل کریں اگر دستیاب ہو"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warn(f'ٹرانسفارم حاصل نہیں کیا جا سکا: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('سینسر پروسیسر نوڈ بند ہو گیا')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## عام مسائل

- **شور ماڈلنگ**: غلط شور پیرامیٹر الگورتھم کو نقصان پہنچا سکتے ہیں جو سیمولیشن میں کام کرتے ہیں لیکن حقیقی ہارڈ ویئر پر ناکام ہو جاتے ہیں
- **کوآرڈینیٹ فریم کے مسائل**: غلط TF ٹرانسفارم سینسر ڈیٹا کو غلط کوآرڈینیٹ سسٹم میں تشریح کا سبب بن سکتے ہیں
- **کارکردگی کے مسائل**: زیادہ ریزولوشن والے سینسرز سیمولیشن کو سست کر سکتے ہیں اگر مناسب طریقے سے آپٹیمائز نہ کیا گیا ہو
- **کیلیبریشن کے مسائل**: غلط کیمرہ انٹرنسکس یا ایکسٹرنسکس ادراک کی غلطی کا سبب بن سکتے ہیں
- **ہم آہنگی کے مسائل**: مختلف سینسر اپ ڈیٹ شرحس ٹائم کے مسائل کا سبب بن سکتے ہیں سینسر فیوژن میں

## چیک پوائنٹس / مینی ایکسائزز

1. Gazebo میں حقیقی شور پیرامیٹر کے ساتھ LiDAR سینسر کنفیگر اور ٹیسٹ کریں
2. یونیٹی میں مناسب ڈسٹورشن ماڈلنگ کے ساتھ ایک کیمرہ سینسر نافذ کریں
3. حقیقی ڈریفٹ اور بائس کی خصوصیات کے ساتھ IMU سیمولیشن تیار کریں
4. ROS 2 نوڈ میں سیمولیٹڈ سینسر ڈیٹا کو پروسیس کریں تاکہ رکاوٹس کا پتہ لگایا جا سکے
5. سیمولیشن کی درستگی کی توثیق کریں نظریاتی ماڈلز کے ساتھ موازنہ کر کے

## حوالہ جات

- [جیزبو سینسر دستاویزات](http://gazebosim.org/tutorials?tut=sensor_noise)
- [ROS 2 سینسر پیغام اقسام](https://docs.ros.org/en/humble/p/sensor_msgs/)
- [کیمرہ کیلیبریشن اور ریکٹیفکیشن](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [سینسر فیوژن تکنیکیں](https://ieeexplore.ieee.org/document/8794278)
- [روبوٹکس میں LiDAR سیمولیشن](https://www.mdpi.com/1424-8220/20/1/233)