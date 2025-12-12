---
title: "چیپٹر 2: جیزبو فزکس سیمولیشن"
sidebar_label: "چیپٹر 2: جیزبو فزکس سیمولیشن"
---

# چیپٹر 2: جیزبو فزکس سیمولیشن

## جائزہ

یہ چیپٹر جیزبو پر مرکوز ہے، روبوٹکس کے لیے اعلی درجے کا فزکس سیمولیشن ماحول۔ آپ سیکھیں گے کہ درست تصادم کا پتہ لگانے، رابطہ کی قوتیں، اور میٹریل کی خصوصیات کے ساتھ حقیقی فزکس سیمولیشن کیسے تیار کریں۔ ہم جیزبو کے مختلف فزکس انجن، ورلڈ تعمیر کے طریقے، اور ROS 2 کے ساتھ انضمام کو تلاش کریں گے۔ اس چیپٹر کے اختتام تک، آپ روبوٹکس سسٹم کے لیے جامع فزکس-مبنی سیمولیشن تیار کرنے کے قابل ہوں گے۔

جیزبو ایک طاقتور 3D سیمولیشن ماحول ہے جو سخت جسم کی فزکس، سینسر ڈیٹا، اور ماحولی بات چیت کی درست سیمولیشن فراہم کرتا ہے۔ یہ حقیقی انڈور اور آؤٹ ڈور ماحول میں پیچیدہ روبوٹس کی سیمولیشن کے لیے ایک مضبوط پلیٹ فارم فراہم کرتا ہے، جو ہیومنوائڈ روبوٹ کی ترقی کے لیے ضروری ہے۔ جیزبو کی فزکس کی صلاحیتوں کو سمجھنا ایسی سیمولیشن تیار کرنے کے لیے اہم ہے جو حقیقی دنیا کے روبوٹ کے رویے کی درست نمائندگی کرے۔

## سیکھنے کے اہداف

- جیزبو کی معماری اور فزکس انجن کی صلاحیتوں کو سمجھنا
- مناسب تصادم اور وژوئل ماڈلز کے ساتھ حقیقی فزکس سیمولیشن تیار کرنا
- مختلف اشیاء اور زمینوں کے ساتھ پیچیدہ ماحول تیار کرنا
- ROS 2 کے ساتھ جیزبو کو ضم کرنا تاکہ سیمولیشن ورک فلو بے داغ ہو
- حقیقی روبوٹ-ماحول کی بات چیت کے لیے فزکس پیرامیٹر کنفیگر کرنا
- جیزبو ماحول میں سینسر سیمولیشن نافذ کرنا

## کلیدی تصورات

### جیزبو معماری

جیزبو کی معماری میں ایک فزکس انجن، رینڈرنگ انجن، اور سینسر سسٹم ہوتا ہے جو حقیقی سیمولیشن فراہم کرنے کے لیے ایک ساتھ کام کرتے ہیں۔ ماڈیولر ڈیزائن مختلف فزکس انجن (ODE، بُلیٹ، DART) اور رینڈرنگ بیک اینڈس کی اجازت دیتا ہے۔

### فزکس انجن پیرامیٹر

اہم فزکس پیرامیٹر میں گریویٹی، ٹائم سٹیپ، سالور ترتیبات، اور تصادم کا پتہ لگانے کے پیرامیٹر شامل ہیں۔ انہیں احتیاط سے کنفیگر کرنا ضروری ہے تاکہ درستگی اور کارکردگی کو متوازن کیا جا سکے۔

### SDF (سیمولیشن ڈسکرپشن فارمیٹ)

SDF جیزبو کا مقامی فارمیٹ ہے جو روبوٹس، ماحول، اور سیمولیشن پیرامیٹر کی وضاحت کے لیے استعمال ہوتا ہے۔ یہ URDF کی صلاحیتوں کو وسعت دیتا ہے تاکہ فزکس، سینسرز، اور سیمولیشن کے لیے مخصوص پلگ ان شامل ہو سکیں۔

### ورلڈ تعمیر

حقیقی ماحول تیار کرنا اس بات کی وضاحت کو شامل کرتا ہے کہ زمین، اشیاء، لائٹنگ، اور فزکس کی خصوصیات جو منصوبہ بند ایپلی کیشن منظر کی درست نمائندگی کریں۔

## تکنیکی گہرائی

### جیزبو فزکس انجن

جیزبو متعدد فزکس انجن کی حمایت کرتا ہے، ہر ایک کے مختلف خصوصیات ہیں:

**ODE (پن ڈائنا مکس انجن)**:
- پرانے جیزبو ورژن میں ڈیفالٹ فزکس انجن
- جنرل پرپز سیمولیشن کے لیے اچھا
- پیچیدہ جوائنٹ اقسام اور پابندیوں کی حمایت کرتا ہے
- اچھی طرح سے ٹیسٹ کیا گیا اور مستحکم

**بُلیٹ**:
- زیادہ کارکردگی والا فزکس انجن
- حقیقی وقت سیمولیشن کے لیے اچھا
- اعلی درجے کے تصادم کا پتہ لگانے کی حمایت کرتا ہے
- گیم ترقی میں استعمال ہوتا ہے

**DART (ڈائنا مکس اینیمیشن اور روبوٹکس ٹول کٹ)**:
- جدید فزکس انجن جس میں اعلی درجے کی خصوصیات ہیں
- پیچیدہ کنیمیٹک چینز کو بہتر طریقے سے سنبھالتا ہے
- زیادہ درست رابطہ سیمولیشن
- ہیومنوائڈ روبوٹس کے لیے اچھا

### فزکس پیرامیٹر کنفیگریشن

سیمولیشن کی معیار کو متاثر کرنے والے اہم فزکس پیرامیٹر:

**ٹائم سٹیپ**: سیمولیشن کا سٹیپ سائز درستگی اور استحکام کو متاثر کرتا ہے۔ چھوٹے سٹیپس بہتر درستگی فراہم کرتے ہیں لیکن زیادہ کمپیو ٹیشن کی ضرورت ہوتی ہے۔

**حقیقی وقت کا عنصر**: یہ کنٹرول کرتا ہے کہ سیمولیشن حقیقی وقت کے مقابلے میں کتنا تیز چلتا ہے۔ 1.0 کا عنصر کا مطلب ہے کہ سیمولیشن حقیقی وقت کے برابر رفتار سے چلتا ہے۔

**سالور پیرامیٹر**: ان میں خامی کم کرنے کے پیرامیٹر (ERP) اور کنکشن فورس مکسنگ (CFM) شامل ہیں جو کنکشن استحکام کو متاثر کرتے ہیں۔

**تصادم کا پتہ لگانا**: پیرامیٹر جو یہ تعین کرتے ہیں کہ رابطے کیسے پتہ لگائے جاتے ہیں اور پروسیس کیے جاتے ہیں۔

### SDF بمقابلہ URDF

جیسے URDF روبوٹ کنیمیٹکس کی وضاحت کرتا ہے، SDF اسے سیمولیشن-مخصوص عناصر شامل کرنے کے لیے وسعت دیتا ہے:

- فزکس کی خصوصیات (ماس، اناشیا، فریکشن)
- سینسر کی وضاحتیں اور پلگ انز
- وژوئل اور تصادم جیومیٹریز
- ماحول کی وضاحتیں
- سیمولیشن پلگ انز اور کنٹرولرز

### جیزبو معماری (متن ڈائیگرام)

```
+-----------------------------------------------------------+
|                        جیزبو سسٹم                        |
|                                                           |
|  +----------------+    +----------------+    +----------+ |
|  |   فزکس         |    |   رینڈرنگ       |    |   سینسر  | |
|  |   انجن          |    |   انجن          |    |   سسٹم  | |
|  |   (ODE/بُلیٹ/   |    |   (OGRE/       |    |   (رے،  | |
|  |   DART)         |    |   OpenGL)       |    |   کیمرہ،| |
|  +----------------+    +----------------+    |   IMU)   | |
|         |                       |             +----------+ |
|         | فزکس اپ ڈیٹس         | رینڈرنگ     |          |
|         | اور کنکشنز            | اور وژولائزیشن|         |
|         v                       v             |          |
|  +----------------+    +----------------+    |          |
|  |   تصادم کا      |    |   GUI اور        |    |          |
|  |   پتہ لگانا     |    |   وژولائزیشن    |    |          |
|  |   اور رابطہ     |    |   (RViz/جیزبو)  |    |          |
|  +----------------+    +----------------+    |          |
|         |                       |             |          |
|         +-----------------------+-------------+----------+
|                           |                               |
|                    +------v------+                        |
|                    |   ROS 2      |                        |
|                    |   انٹرفیس   |                        |
|                    |   (پلگ انز)  |                        |
|                    +--------------+                        |
+-----------------------------------------------------------+
```

## کوڈ کی مثالیں

### فزکس کنفیگریشن کے ساتھ بنیادی جیزبو ورلڈ

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physics_world">
    <!-- فزکس انجن کنفیگریشن -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- معیاری ماڈلز شامل کریں -->
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- روبوٹ ماڈل -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

    <!-- پیچیدہ ماحولی اشیاء -->
    <model name="table">
      <pose>2 0 0.4 0 0 0</pose>
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

    <!-- مختلف جسمانی خصوصیات والی اشیاء -->
    <model name="friction_ball">
      <pose>-1 1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>10.0</mu>
                <mu2>10.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.001</iyy>
            <iyz>0.0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="bouncy_ball">
      <pose>1 1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <surface>
            <bounce>
              <restitution_coefficient>0.8</restitution_coefficient>
              <threshold>100000</threshold>
            </bounce>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.001</iyy>
            <iyz>0.0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

### جیزبو پلگ انز کے ساتھ روبوٹ ماڈل

```xml
<?xml version="1.0"?>
<robot name="gazebo_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- جسمانی خصوصیات کے ساتھ بیس لنک -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- جیزبو-مخصوص خصوصیات -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- ایکٹو ایٹر کے ساتھ ریوولوٹ جوائنٹ -->
  <joint name="base_to_upper_arm" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="upper_arm">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <gazebo reference="upper_arm">
    <material>Gazebo/Red</material>
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
  </gazebo>

  <!-- کنٹرول کے لیے ٹرانسمیشن کے ساتھ جوائنٹ -->
  <transmission name="trans_upper_arm">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="base_to_upper_arm">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor_upper_arm">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- ROS 2 انضمام کے لیے جیزبو پلگ انز -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/my_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- LiDAR سینسر پلگ ان -->
  <gazebo reference="base_link">
    <sensor type="ray" name="laser_scanner">
      <pose>0.2 0 0.1 0 0 0</pose>
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
      <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>laser_scanner_frame</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU سینسر پلگ ان -->
  <gazebo reference="base_link">
    <sensor type="imu" name="imu_sensor">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
        <topicName>imu</topicName>
        <bodyName>base_link</bodyName>
        <frameName>base_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### جیزبو انٹر ایکشن کے لیے ROS 2 نوڈ

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np


class GazeboRobotController(Node):
    """نیا جو جیزبو سیمولیشن کے ساتھ بات چیت کرتا ہے"""

    def __init__(self):
        super().__init__('gazebo_robot_controller')

        # روبوٹ کنٹرول کے لیے شائع کنندہ
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/my_robot/cmd_vel', 10)

        # جوائنٹ کنٹرول کے لیے شائع کنندہ (اگر جوائنٹ اسٹیٹ کنٹرول استعمال کر رہے ہیں)
        self.joint_cmd_publisher = self.create_publisher(
            Float64MultiArray, '/my_robot/joint_commands', 10)

        # سینسر ڈیٹا کے لیے سبسکرائبر
        self.scan_subscription = self.create_subscription(
            LaserScan, '/my_robot/scan', self.scan_callback, 10)

        self.imu_subscription = self.create_subscription(
            Imu, '/my_robot/imu', self.imu_callback, 10)

        # کنٹرول لوپ کے لیے ٹائمر
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # روبوٹ اسٹیٹ متغیرات
        self.scan_data = None
        self.imu_data = None
        self.obstacle_detected = False

        self.get_logger().info('جیزبو روبوٹ کنٹرولر انشائیلائز ہوا')

    def scan_callback(self, msg):
        """جیزبو سے لیزر اسکین ڈیٹا پروسیس کریں"""
        self.scan_data = msg
        # روبوٹ کے سامنے رکاوٹوں کی جانچ کریں
        if len(msg.ranges) > 0:
            # سامنے کے 30 ڈگری چیک کریں
            front_ranges = msg.ranges[len(msg.ranges)//2-15:len(msg.ranges)//2+15]
            valid_ranges = [r for r in front_ranges if msg.range_min <= r <= msg.range_max]
            if valid_ranges:
                min_distance = min(valid_ranges)
                self.obstacle_detected = min_distance < 1.0  # 1 میٹر کی حد
                self.get_logger().debug(f'سامنے کا کم از کم فاصلہ: {min_distance:.2f}م')

    def imu_callback(self, msg):
        """جیزبو سے IMU ڈیٹا پروسیس کریں"""
        self.imu_data = msg
        # ضرورت کے مطابق اورینٹیشن یا ایکسلریشن ڈیٹا نکالیں
        self.get_logger().debug(f'IMU اورینٹیشن: ({msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z})')

    def control_loop(self):
        """سیمولیشن میں روبوٹ کے لیے مرکزی کنٹرول لوپ"""
        cmd = Twist()

        if self.obstacle_detected:
            # رکیں اور رکاوٹ سے بچنے کے لیے موڑیں
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # دائیں طرف موڑیں
            self.get_logger().info('رکاوٹ کا پتہ چلا - موڑ رہا ہے')
        else:
            # آگے بڑھیں
            cmd.linear.x = 0.3  # 0.3 میٹر/سیکنڈ آگے
            cmd.angular.z = 0.0
            self.get_logger().info('آگے بڑھ رہا ہے')

        # رفتار کمانڈ شائع کریں
        self.cmd_vel_publisher.publish(cmd)

    def publish_joint_commands(self):
        """جوائنٹ پوزیشن کمانڈز شائع کریں اگر ضرورت ہو"""
        cmd = Float64MultiArray()
        # مثال: جوائنٹ پوزیشنز [0.1, 0.2, 0.3] ریڈینز بھیجیں
        cmd.data = [0.1, 0.2, 0.3]
        self.joint_cmd_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    controller = GazeboRobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('جیزبو کنٹرولر بند ہو گیا')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## عام مسائل

- **فزکس کی عدم استحکامی**: غلط فزکس پیرامیٹر سیمولیشن کی عدم استحکامی یا غیر حقیقی رویے کا سبب بن سکتے ہیں
- **تصادم مش کی معیار**: غلط تصادم مشز غلط فزکس بات چیت کا سبب بن سکتی ہیں
- **ٹائم سٹیپ کے مسائل**: بہت بڑے ٹائم سٹیپس غلط فزکس کا سبب بن سکتے ہیں، بہت چھوٹے کارکردگی کے مسائل کا سبب بن سکتے ہیں
- **پلگ ان کنفیگریشن**: غلط جیزبو پلگ ان کنفیگریشن ROS 2 انضمام کو مناسب طریقے سے روک سکتا ہے
- **سالور پیرامیٹر**: غلط سالور کنفیگریشن کنکشن کی خلاف ورزیوں اور غیر مستحکم سیمولیشن کا سبب بن سکتی ہے

## چیک پوائنٹس / مینی ایکسائزز

1. مختلف جسمانی خصوصیات (فریکشن، باؤنس) کے ساتھ ایک جیزبو ورلڈ تیار کریں
2. مناسب اناشیا کی خصوصیات اور فزکس پلگ انز کے ساتھ ایک روبوٹ ماڈل نافذ کریں
3. رکاوٹوں والے پیچیدہ ماحول میں روبوٹ کی سیمولیشن کریں
4. مختلف فزکس انجن کنفیگر کریں اور ان کے رویے کا موازنہ کریں
5. جیزبو سے سینسر ڈیٹا کو ROS 2 کنٹرول نوڈ میں ضم کریں

## حوالہ جات

- [جیزبو فزکس دستاویزات](http://gazebosim.org/tutorials?tut=physics)
- [SDF سپیسیفکیشن](http://sdformat.org/)
- [جیزبو ROS انضمام](http://gazebosim.org/tutorials/?tut=ros2_overview)
- [فزکس سیمولیشن بہترین طریقے](http://gazebosim.org/tutorials?tut=guided_b1)
- [جیزبو کے ساتھ روبوٹکس سیمولیشن](https://arxiv.org/abs/1807.00885)