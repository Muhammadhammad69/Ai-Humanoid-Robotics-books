---
title: "چیپٹر 3: rclpy کے ساتھ پائی تھون پروگرامنگ"
sidebar_label: "چیپٹر 3: rclpy کے ساتھ پائی تھون پروگرامنگ"
---

# چیپٹر 3: rclpy کے ساتھ پائی تھون پروگرامنگ

## جائزہ

یہ چیپٹر پائی تھون اور rclpy کلائنٹ لائبریری کا استعمال کرتے ہوئے ROS 2 نوڈس ترقی دینے پر مرکوز ہے۔ آپ سیکھیں گے کہ پیچیدہ روبوٹک سسٹم میں حصہ لینے والے پائی تھون-مبنی ROS 2 نوڈس کو کیسے تیار، ساخت اور چلایا جاتا ہے۔ ہم اہم نمونے، بہترین طریقے، اور مضبوط پائی تھون نوڈس تیار کرنے کے اعلی درجے کے طریقے کو کور کریں گے جو حقیقی دنیا کی روبوٹک ایپلی کیشنز کو سنبھال سکتے ہیں بشمول سینسر پروسیسنگ، کنٹرول سسٹم، اور دوسرے نوڈس کے ساتھ مواصلت۔

پائی تھون روبوٹکس ترقی کے لیے سب سے مقبول زبانوں میں سے ایک ہے اس کی سادگی، وسیع لائبریریز، اور سائنسی کمپیو ٹنگ کی مضبوط حمایت کی وجہ سے۔ rclpy لائبریری ROS 2 کے بنیادی فعالیت کے لیے ایک پائی تھون انٹرفیس فراہم کرتی ہے، جو روبوٹک ایپلی کیشنز کی تیز پروٹو ٹائپنگ اور ترقی کو فعال کرتا ہے جبکہ ROS 2 کے تقسیم شدہ معماری کے فوائد برقرار رکھتا ہے۔

## سیکھنے کے اہداف

- پائی تھون-مبنی ROS 2 ترقی کے لیے rclpy کے بنیادیات ماسٹر کریں
- مختلف مواصلت کے نمونے (ٹاپکس، سروسز، ایکشنز) نافذ کرنے والے نوڈس تیار کریں
- مناسب نوڈ لائف سائیکل مینجمنٹ اور ریسورس ہینڈلنگ نافذ کریں
- جواب دہ نوڈ کے رویے کے لیے تھریڈنگ اور async نمونے لاگو کریں
- پائی تھون نوڈس میں مؤثر طریقے سے ROS 2 پیرامیٹرز اور لاگنگ کا استعمال کریں
- برقرار رکھنے اور دوبارہ استعمال کے قابل کے لیے پیچیدہ پائی تھون نوڈس کو ساخت دیں

## کلیدی تصورات

### rclpy معماری

rclpy ROS 2 کے لیے پائی تھون کلائنٹ لائبریری ہے جو ROS کلائنٹ لائبریری (rcl) کے لیے پائی تھون بائنڈنگس فراہم کرتی ہے۔ یہ DDS مڈل ویئر کے ساتھ کم سطح کی مواصلت کو سنبھالتی ہے جبکہ نوڈس، شائع کنندہ، سبسکرائبرز، اور دیگر ROS 2 کی تعمیرات کے لیے ایک پائی تھون کے مطابق انٹرفیس فراہم کرتی ہے۔

### نوڈ کی ساخت اور لائف سائیکل

پائی تھون میں ROS 2 نوڈس ایک معیاری ساخت پر عمل کرتے ہیں جس میں شروع کرنا، انجام دینا، اور صاف کرنا کے ادوار شامل ہیں۔ ریسورس ہینڈلنگ اور سسٹم کی استحکام کے لیے مناسب لائف سائیکل مینجمنٹ انتہائی اہم ہے، خاص طور پر طویل چلنے والی روبوٹک ایپلی کیشنز میں۔

### غیر ہم وقت پروگرامنگ

موثر ROS 2 نوڈس اکثر متعدد کاموں کو متوازی طور پر سنبھالنے کے لیے غیر ہم وقت پروگرامنگ نمونے استعمال کرتے ہیں، جیسے سروس کی درخواستوں کے جواب دینے کے دوران سینسر ڈیٹا کو پروسیس کرنا اور پیرامیٹرز اپ ڈیٹ کرنا۔

### پیرامیٹر مینجمنٹ

ROS 2 ایک پیرامیٹر سسٹم فراہم کرتا ہے جو نوڈس کو رن ٹائم پر کنفیگر کرنے کی اجازت دیتا ہے، جس سے انہیں مختلف روبوٹک پلیٹ فارمز اور منظار میں زیادہ لچک اور دوبارہ استعمال کے قابل بناتا ہے۔

## تکنیکی گہرائی

### rclpy بنیادیات

rclpy لائبریری ROS 2 فعالیت تک پائی تھون رسائی فراہم کرتی ہے ایک کلاسز اور فنکشنز کے سیٹ کے ذریعے جو ROS 2 تصورات کو عکسیں کرتے ہیں:

**نوڈ کلاس**: پائی تھون میں تمام ROS 2 نوڈس کے لیے بیس کلاس۔ یہ شائع کنندہ، سبسکرائبرز، سروسز، اور دیگر ROS 2 اداروں کو تیار کرنے کے لیے طریقے فراہم کرتی ہے۔

**انشائیلائزیشن**: `rclpy.init()` فنکشن ROS 2 کلائنٹ لائبریری کو انشائیلائز کرتا ہے اور کوئی بھی نوڈ تیار کرنے سے پہلے اسے کال کیا جانا چاہیے۔

**اسپننگ**: `rclpy.spin()` فنکشن نوڈ کو چلتا رکھتا ہے اور کال بیکس، پیغامات، اور سروس کی درخواستوں کو پروسیس کرتا ہے۔

### نوڈ لائف سائیکل مینجمنٹ

ایک مناسب طور پر ساخت شدہ پائی تھون نوڈ کو ان لائف سائیکل اقدامات پر عمل کرنا چاہیے:

1. **انشائیلائزیشن**: نوڈ کو سیٹ اپ کریں، شائع کنندہ/سبسکرائبرز تیار کریں، اندرونی حالت کو انشائیلائز کریں
2. **ایکزیکیوشن**: کال بیکس، ٹائمرز، اور دیگر ایونٹس کو پروسیس کریں
3. **کلین اپ**: وسائل کو تباہ کریں اور بند کرنے سے پہلے صاف کریں

### تھریڈنگ اور کنکرنسی

ROS 2 نوڈس مختلف ایکزیکیوشن ماڈلز استعمال کر سکتے ہیں:

- **سنگل-تھریڈڈ**: ڈیفالٹ ایکزیکیوٹر کال بیکس کو متوالی طور پر پروسیس کرتا ہے
- **ملٹی-تھریڈڈ**: MultiThreadedExecutor کال بیکس کو متوازی طور پر پروسیس کرتا ہے
- **کسٹم**: مخصوص استعمال کے معاملات کے لیے کسٹم ایکزیکیوٹرز

### پیرامیٹر ہینڈلنگ

rclpy میں پیرامیٹر نوڈس کو رن ٹائم پر کنفیگر کرنے کی اجازت دیتے ہیں:

- ڈیفالٹ قیمتوں اور پابندیوں کے ساتھ اعلان کیا گیا
- کمانڈ لائن، لاؤنچ فائلز، یا پیرامیٹر سرور کے ذریعے قابل رسائی
- کال بیکس کے ساتھ رن ٹائم کے دوران تبدیل کرنا ممکن

### پائی تھون نوڈ معماری (متن ڈائیگرام)

```
+-----------------------------------------------------------+
|                   پائی تھون نوڈ معماری                   |
|                                                           |
|  +-------------------+     +-------------------------+    |
|  |   rclpy لائبریری  |---->|   ROS 2 مڈل ویئر       |    |
|  |   (پائی تھون)     |     |   (DDS ایمپلیمنٹیشن)  |    |
|  +-------------------+     +-------------------------+    |
|           |                           |                    |
|           v                           v                    |
|  +-------------------+     +-------------------------+    |
|  |   نوڈ کلاس        |     |   ٹاپک/سروس/ایکشن      |    |
|  |   (یوزر کوڈ)     |---->|   مواصلت کی انفراسٹرکچر|    |
|  |                   |     |                       |    |
|  | - شائع کنندہ      |     |                       |    |
|  | - سبسکرائبر       |     |                       |    |
|  | - سروسز           |     |                       |    |
|  | - ایکشنز         |     |                       |    |
|  | - پیرامیٹر       |     |                       |    |
|  | - ٹائمرز         |     |                       |    |
|  +-------------------+     +-------------------------+    |
|           |                           |                    |
|           v                           v                    |
|  +-------------------+     +-------------------------+    |
|  |   کال بیک کیو    |---->|   پیغام کیو            |    |
|  |   (ایکزیکیوٹر)   |     |   (DDS/RMW)           |    |
|  |                   |     |                       |    |
|  | - ٹائمر کال بیکس |     | - آنے والے پیغامات   |    |
|  | - سبسکرپشن       |     | - سروس کی درخواستیں   |    |
|  |   کال بیکس       |     | - ایکشن اہداف        |    |
|  | - سروس           |     |                       |    |
|  |   کال بیکس       |     |                       |    |
|  +-------------------+     +-------------------------+    |
+-----------------------------------------------------------+
```

## کوڈ کی مثالیں

### پیرامیٹر کے ساتھ بنیادی نوڈ ساخت

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ParameterizedNode(Node):
    """پیرامیٹر کے استعمال کو ظاہر کرنے والی مثال نوڈ"""

    def __init__(self):
        super().__init__('parameterized_node')

        # ڈیفالٹ قیمتوں کے ساتھ پیرامیٹر اعلان کریں
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('sensor_range', 10.0)

        # پیرامیٹر کی قیمتوں تک رسائی حاصل کریں
        self.publish_rate = self.get_parameter('publish_rate').value
        self.robot_name = self.get_parameter('robot_name').value
        self.sensor_range = self.get_parameter('sensor_range').value

        # QoS پروفائل کے ساتھ شائع کنندہ تیار کریں
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.publisher = self.create_publisher(String, 'robot_status', qos_profile)

        # پیرامیٹر کے مطابق ٹائمر تیار کریں
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info(
            f'نوڈ انشائیلائز ہوا: {self.robot_name}, '
            f'شائع کرنا کی شرح: {self.publish_rate}Hz, '
            f'سینسر کی حد: {self.sensor_range}م'
        )

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.robot_name} {self.publish_rate}Hz پر کام کر رہا ہے'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('یوزر کے ذریعے نوڈ میں مداخلت کی گئی')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### متعدد مواصلت کے نمونوں کے ساتھ پیچیدہ نوڈ

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from example_interfaces.action import Fibonacci
from example_interfaces.srv import SetBool
import threading
import time


class ComplexRobotController(Node):
    """متعدد مواصلت کے نمونوں کو ظاہر کرنے والی اعلی درجے کی نوڈ"""

    def __init__(self):
        super().__init__('complex_robot_controller')

        # شائع کنندہ
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.position_publisher = self.create_publisher(Float64, 'current_position', 10)

        # سبسکرائبر
        self.odometry_subscriber = self.create_subscription(
            Float64, 'odometry', self.odometry_callback, 10)

        # سروس سرور
        self.service = self.create_service(
            SetBool, 'enable_controller', self.service_callback)

        # ایکشن سرور
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_fibonacci,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        # اندرونی حالت
        self.controller_enabled = True
        self.current_position = 0.0
        self.active_goals = {}

        # پوزیشن اپ ڈیٹس کے لیے ٹائمر
        self.position_timer = self.create_timer(0.1, self.publish_position)

        self.get_logger().info('پیچیدہ روبوٹ کنٹرولر انشائیلائز ہوا')

    def odometry_callback(self, msg):
        """آنے والے اودومیٹری ڈیٹا کو سنبھالیں"""
        self.current_position = msg.data
        if self.controller_enabled:
            self.get_logger().debug(f'پوزیشن اپ ڈیٹ ہوئی: {self.current_position}')

    def publish_position(self):
        """موجودہ پوزیشن کو مسلسل شائع کریں"""
        msg = Float64()
        msg.data = self.current_position
        self.position_publisher.publish(msg)

    def service_callback(self, request, response):
        """فعال/غیر فعال سروس کی درخواستوں کو سنبھالیں"""
        self.controller_enabled = request.data
        response.success = True
        response.message = f'کنٹرولر {"فعال" if self.controller_enabled else "غیر فعال"}'
        self.get_logger().info(f'کنٹرولر {response.message}')
        return response

    def goal_callback(self, goal_request):
        """اهداف قبول یا مسترد کریں"""
        self.get_logger().info(f'ہدف کی درخواست موصول ہوئی: {goal_request}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """ہدف کی منسوخی کی درخواستوں کو قبول یا مسترد کریں"""
        self.get_logger().info('منسوخی کی درخواست موصول ہوئی')
        return CancelResponse.ACCEPT

    async def execute_fibonacci(self, goal_handle):
        """فیبونیچی ایکشن انجام دیں"""
        self.get_logger().info('ہدف انجام دیا جا رہا ہے...')

        # فیڈ بیک اور نتیجہ کے پیغامات تیار کریں
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # فیبونیچی ترتیب تیار کریں
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.sequence = feedback_msg.sequence
                self.get_logger().info('ہدف منسوخ ہو گیا')
                return result_msg

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            # فیڈ بیک شائع کریں
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().debug(f'فیڈ بیک: {feedback_msg.sequence[-1]}')

            # کام کی شبیہ سازی کے لیے سلیپ
            time.sleep(0.5)

        goal_handle.succeed()
        result_msg.sequence = feedback_msg.sequence
        self.get_logger().info(f'ہدف کامیابی کے ساتھ مکمل ہوا: {result_msg.sequence}')
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    node = ComplexRobotController()

    # متوازی پروسیسنگ کے لیے ملٹی-تھریڈڈ ایکزیکیوٹر استعمال کریں
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('یوزر کے ذریعے نوڈ میں مداخلت کی گئی')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### پیچیدہ نوڈ کے لیے لاؤنچ فائل

```python
# complex_robot_controller_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # لاؤنچ آرگومنٹس اعلان کریں
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot',
        description='روبوٹ کا نام'
    )

    # لاؤنچ کنفیگریشن حاصل کریں
    robot_name = LaunchConfiguration('robot_name')

    # پیچیدہ روبوٹ کنٹرولر نوڈ تیار کریں
    controller_node = Node(
        package='my_robot_package',
        executable='complex_robot_controller',
        name='complex_controller',
        parameters=[
            {'robot_name': robot_name},
            {'publish_rate': 10.0},
            {'sensor_range': 5.0}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_mux/input/teleop'),
            ('odometry', 'odom')
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        controller_node
    ])
```

## عام مسائل

- **تھریڈنگ کے مسائل**: کال بیکس کو غلط طریقے سے ہینڈل کرنا ریس کنڈیشنز کا سبب بن سکتا ہے۔ مشترکہ ڈیٹا تک رسائی کے دوران مناسب تال میل کے طریقے استعمال کریں۔
- **ریسورس مینجمنٹ**: نوڈس اور ان کے اجزاء کو مناسب طریقے سے تباہ کرنا بھولنا طویل چلنے والے سسٹم میں میموری لیکس کا سبب بن سکتا ہے۔
- **پیرامیٹر کی توثیق**: پیرامیٹر کی قیمتوں کی توثیق نہ کرنا رن ٹائم کی غلطیوں یا غیر متوقع رویے کا سبب بن سکتا ہے۔
- **ایکسیپشن ہینڈلنگ**: کال بیکس میں غیر ہینڈل شدہ ایکسیپشنز پورے نوڈ کو کریش کر سکتے ہیں۔ ہمیشہ مناسب خامی کے ہینڈلنگ کو نافذ کریں۔
- **ٹائمر کی درستی**: سسٹم لوڈ اور شیڈولنگ لیٹنسی پر غور کیے بغیر ٹائم کریٹیکل ایپلی کیشنز کے لیے ٹائمرز کا استعمال کرنا۔

## چیک پوائنٹس / مینی ایکسائزز

1. ایک پائی تھون نوڈ تیار کریں جو سینسر ڈیٹا کو سبسکرائب کرتا ہے اور مناسب QoS ترتیبات کا استعمال کرتے ہوئے پروسیس کردہ معلومات شائع کرتا ہے
2. ایک پیرامیٹرائزڈ نوڈ نافذ کریں جو رن ٹائم پیرامیٹرز کے مطابق اس کا رویہ تبدیل کرتا ہے
3. ایک نوڈ تیار کریں جو مختلف قسم کی فعالیت فراہم کرنے کے لیے سروسز اور ایکشنز دونوں کا استعمال کرتا ہے
4. ایک لاؤنچ فائل تیار کریں جو مختلف پیرامیٹر کنفیگریشنز کے ساتھ آپ کے نوڈ کے متعدد انسٹانسز شروع کرتی ہے
5. ایک نوڈ تیار کریں جو خامیوں کو مناسب طریقے سے سنبھالتا ہے اور جب اجزاء ناکام ہو جاتے ہیں تو بے عیب طریقے سے کم کام کرتا ہے

## حوالہ جات

- [rclpy دستاویزات](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 پائی تھون نوڈ ٹیوٹوریل](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 پیرامیٹرز گائیڈ](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-in-a-class-CPP.html)
- [ROS 2 لاؤنچ سسٹم](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)