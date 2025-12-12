---
title: "چیپٹر 4: URDF روبوٹ ماڈلنگ"
sidebar_label: "چیپٹر 4: URDF روبوٹ ماڈلنگ"
---

# چیپٹر 4: URDF روبوٹ ماڈلنگ

## جائزہ

یہ چیپٹر URDF (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ) کو تلاش کرتا ہے، ROS میں روبوٹ ماڈلز کی نمائندگی کے لیے معیاری XML-مبنی فارمیٹ۔ آپ سیکھیں گے کہ وہ تفصیلی روبوٹ کی تفصیل کیسے تیار کریں جس میں کنیمیٹک چینز، بصری اور تصادم کی خصوصیات، اور جسمانی پیرامیٹر شامل ہوں۔ URDF سیمولیشن، وژولائزیشن، اور روبوٹک سسٹم کے کنٹرول کے لیے ضروری ہے، خاص طور پر پیچیدہ کنیمیٹک سٹرکچر والے ہیومنوائڈ روبوٹس کے لیے۔

URDF روبوٹ کی جسمانی اور کنیمیٹک خصوصیات کو ایک سٹرکچرڈ طریقے سے بیان کرنے کی اجازت دیتا ہے جسے مختلف ROS ٹولز اور سیمولیشن ماحول کے ذریعے استعمال کیا جا سکتا ہے۔ URDF کو سمجھنا ایسے روبوٹس تیار کرنے کے لیے انتہائی ضروری ہے جنہیں مناسب طریقے سے سیمولیٹ کیا جا سکے، RViz میں وژولائز کیا جا سکے، اور ROS-مبنی ٹولز اور الگورتھم کے استعمال سے کنٹرول کیا جا سکے۔

## سیکھنے کے اہداف

- URDF فائلز کی سٹرکچر اور اجزاء کو سمجھنا
- مناسب کنیمیٹک چینز کے ساتھ مکمل روبوٹ ماڈلز تیار کرنا
- روبوٹ لنکس کے لیے بصری اور تصادم کی خصوصیات کی وضاحت کرنا
- جسمانی خصوصیات شامل کرنا بشمول ماس، اناشیا، اور فریکشن
- Gazebo جیسے سیمولیشن ماحول کے ساتھ URDF ماڈلز کا انضمام
- ROS ٹولز کا استعمال کرتے ہوئے URDF ماڈلز کی توثیق اور ڈیبگ کرنا

## کلیدی تصورات

### URDF سٹرکچر

URDF ایک XML-مبنی فارمیٹ ہے جو روبوٹس کو جوائنٹس کے ذریعے منسلک لنکس کے مجموعہ کے طور پر بیان کرتا ہے۔ سٹرکچر میں رینڈرنگ کے لیے بصری عناصر، فزکس سیمولیشن کے لیے تصادم عناصر، اور ڈائنامکس کیلکولیشنز کے لیے جسمانی خصوصیات شامل ہیں۔

### لنکس اور جوائنٹس

لنکس روبوٹ میں سخت جسم کی نمائندگی کرتے ہیں، جبکہ جوائنٹس لنکس کے درمیان کنکشنز اور ڈیگریز آف فریڈم کی وضاحت کرتے ہیں۔ یہ مجموعہ کنیمیٹک چین تشکیل دیتا ہے جو یہ تعین کرتا ہے کہ روبوٹ کیسے حرکت کرتا ہے۔

### بصری اور تصادم کی خصوصیات

بصری عناصر یہ وضاحت کرتے ہیں کہ روبوٹ وژولائزیشن ٹولز میں کیسا نظر آتا ہے، جبکہ تصادم عناصر فزکس سیمولیشن اور تصادم کے پتہ لگانے کے لیے استعمال ہونے والی شکلوں کی وضاحت کرتے ہیں۔ انہیں کارکردگی اور ظہور کو بہتر بنانے کے لیے مختلف رکھا جا سکتا ہے۔

### اناشیا کی خصوصیات

ماس، مرکز ماس، اور اناشیا ٹینسرز فزکس سیمولیشن اور کنٹرول کے لیے انتہائی اہم ہیں۔ مناسب طریقے سے وضاحت شدہ اناشیا کی خصوصیات سیمولیشن ماحول میں حقیقی روبوٹ کے رویے کو یقینی بناتی ہیں۔

## تکنیکی گہرائی

### URDF فائل سٹرکچر

ایک URDF فائل میں عام طور پر یہ چیزیں ہوتی ہیں:

**روبوٹ عنصر**: روبوٹ کی مکمل تفصیل کو حاوی کرنے والا روٹ عنصر۔

**لنک عناصر**: بصری، تصادم، اور اناشیا کی خصوصیات کے ساتھ سخت جسم کی وضاحت کرتے ہیں۔

**جوائنٹ عناصر**: لنکس کے درمیان کنکشنز کی وضاحت کرتے ہیں مخصوص اقسام (ریوولوٹ، پریزمیٹک، فکسڈ، وغیرہ) اور حدود کے ساتھ۔

**میٹریل عناصر**: رنگ اور ٹیکسچر جیسی بصری ظہور کی خصوصیات کی وضاحت کرتے ہیں۔

### لنک کمپوننٹس

URDF میں ہر لنک میں یہ چیزیں ہو سکتی ہیں:

- **بصری**: یہ وضاحت کرتا ہے کہ لنک وژولائزیشن میں کیسا نظر آتا ہے
- **تصادم**: تصادم کے پتہ لگانے اور فزکس کے لیے شکلیں کیا ہیں
- **اناشیا**: ماس، مرکز ماس، اور اناشیا ٹینسر کی وضاحت کرتا ہے
- **اصل**: اپنے والد کے مقابلے میں لنک کا پوز کی وضاحت کرتا ہے

### جوائنٹ کی اقسام اور خصوصیات

جوائنٹس لنکس کو جوڑتے ہیں اور ان کی ریلیٹو موشن کی وضاحت کرتے ہیں:

- **فکسڈ**: لنکس کے درمیان کوئی ریلیٹو موشن نہیں ہے
- **ریوولوٹ**: حدود کے ساتھ سنگل گھومنے کا ڈیگری آف فریڈم
- **کنٹینیوئس**: حدود کے بغیر سنگل گھومنے کا ڈیگری آف فریڈم
- **پریزمیٹک**: حدود کے ساتھ سنگل ٹرانسلیشنل ڈیگری آف فریڈم
- **فلوٹنگ**: چھ ڈیگریز آف فریڈم (کم استعمال ہوتا ہے)
- **پلینر**: موشن ایک سطح تک محدود ہے

### URDF روبوٹ سٹرکچر (متن ڈائیگرام)

```
روبوٹ (روٹ)
  |
  +-- لنک: base_link
  |   +-- بصری: سلنڈر (جسم)
  |   +-- تصادم: سلنڈر (جسم)
  |   +-- اناشیا: ماس=10kg
  |
  +-- جوائنٹ: base_to_torso (فکسڈ)
  |   |
  |   +-- لنک: torso
  |       +-- بصری: باکس (ٹورسو)
  |       +-- تصادم: باکس (ٹورسو)
  |       +-- اناشیا: ماس=8kg
  |
  +-- جوائنٹ: torso_to_head (ریوولوٹ)
  |   |
  |   +-- لنک: head
  |       +-- بصری: سپیئر (سر)
  |       +-- تصادم: سپیئر (سر)
  |       +-- اناشیا: ماس=2kg
  |
  +-- جوائنٹ: torso_to_left_arm (ریوولوٹ)
      |
      +-- لنک: left_upper_arm
          +-- بصری: سلنڈر (اپر ایرم)
          +-- تصادم: سلنڈر (اپر ایرم)
          +-- اناشیا: ماس=1kg
```

## کوڈ کی مثالیں

### بنیادی URDF روبوٹ ماڈل

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- بیس لنک -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- ٹورسو -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- سر -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.45" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- بائیں بازو -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.25 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_to_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

</robot>
```

### Gazebo انضمام کے ساتھ URDF

```xml
<?xml version="1.0"?>
<robot name="gazebo_integrated_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Gazebo-مخصوص پلگ انز اور مواد شامل کریں -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- بیس لنک کے لیے Gazebo-مخصوص خصوصیات -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- ایکٹو ایٹر کنٹرول والے جوائنٹ -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="10"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- چکر کے لیے Gazebo-مخصوص خصوصیات -->
  <gazebo reference="wheel_link">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>10000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- ڈیفرنشل ڈرائیو کے لیے Gazebo پلگ ان -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>base_to_wheel</left_joint>
      <right_joint>base_to_wheel</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.4</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

</robot>
```

### URDF کو پارس کرنے کے لیے پائی تھون اسکرپٹ

```python
#!/usr/bin/env python3
"""
URDF فائلز کو پارس اور تجزیہ کرنے کے لیے سادہ اسکرپٹ
"""
import xml.etree.ElementTree as ET
from collections import defaultdict


class URDFAnalyzer:
    def __init__(self, urdf_file_path):
        self.tree = ET.parse(urdf_file_path)
        self.root = self.tree.getroot()
        self.robot_name = self.root.get('name')

    def get_links(self):
        """URDF میں تمام لنکس حاصل کریں"""
        links = []
        for link in self.root.findall('link'):
            links.append({
                'name': link.get('name'),
                'visual': link.find('visual') is not None,
                'collision': link.find('collision') is not None,
                'inertial': link.find('inertial') is not None
            })
        return links

    def get_joints(self):
        """URDF میں تمام جوائنٹس حاصل کریں"""
        joints = []
        for joint in self.root.findall('joint'):
            joints.append({
                'name': joint.get('name'),
                'type': joint.get('type'),
                'parent': joint.find('parent').get('link') if joint.find('parent') is not None else None,
                'child': joint.find('child').get('link') if joint.find('child') is not None else None
            })
        return joints

    def print_summary(self):
        """URDF سٹرکچر کا خلاصہ چھاپیں"""
        print(f"روبوٹ: {self.robot_name}")
        print(f"لنکس: {len(self.get_links())}")
        print(f"جوائنٹس: {len(self.get_joints())}")

        print("\nلنکس:")
        for link in self.get_links():
            print(f"  - {link['name']}: بصری={link['visual']}, تصادم={link['collision']}, اناشیا={link['inertial']}")

        print("\nجوائنٹس:")
        for joint in self.get_joints():
            print(f"  - {joint['name']}: {joint['type']} ({joint['parent']} -> {joint['child']})")


def main():
    # مثال کا استعمال
    # analyzer = URDFAnalyzer('path_to_urdf_file.urdf')
    # analyzer.print_summary()
    pass


if __name__ == '__main__':
    main()
```

## عام مسائل

- **اناشیا کی خصوصیات**: غلط ماس یا اناشیا کی قیمتوں کی وجہ سے سیمولیشن میں عدم استحکام یا غیر حقیقی رویہ ہو سکتا ہے
- **جوائنٹ حدود**: مناسب جوائنٹ حدود سیٹ کرنا بھولنا کنیمیٹک غلطیوں یا تصادم کا سبب بن سکتا ہے
- **تصادم بمقابلہ بصری**: بہت پیچیدہ تصادم مشز کا استعمال فزکس سیمولیشن کو سست کر سکتا ہے
- **اصل کی وضاحتیں**: غلط اصل کی وضاحتیں لنکس کو ایک دوسرے کے مقابلے میں غلط طریقے سے پوزیشن کر سکتی ہیں
- **میٹریل کی وضاحتیں**: غائب یا غلط میٹریل کی وضاحتیں وژولائزیشن اور فزکس کی خصوصیات کو متاثر کر سکتی ہیں

## چیک پوائنٹس / مینی ایکسائزز

1. ڈیفرنشل ڈرائیو کے ساتھ ایک سادہ چکروں والے روبوٹ کے لیے URDF ماڈل تیار کریں
2. اپنے روبوٹ ماڈل میں مناسب اناشیا کی خصوصیات شامل کریں اور ان کی توثیق کریں
3. گریپر میکنزم کے ساتھ URDF تیار کریں اور اس کے کنیمیٹکس کو ٹیسٹ کریں
4. مناسب پلگ انز شامل کر کے اپنے URDF کو Gazebo کے ساتھ انضمام کریں
5. RViz کا استعمال کرتے ہوئے اپنے URDF ماڈل کو وژولائز کریں اور کسی بھی بصری مصنوعات کی جانچ کریں

## حوالہ جات

- [URDF دستاویزات](http://wiki.ros.org/urdf)
- [URDF ٹیوٹوریلز](http://wiki.ros.org/urdf/Tutorials)
- [Gazebo URDF انضمام](http://gazebosim.org/tutorials/?tut=ros_urdf)
- [روبوٹ اسٹیٹ پبلشر](http://wiki.ros.org/robot_state_publisher)