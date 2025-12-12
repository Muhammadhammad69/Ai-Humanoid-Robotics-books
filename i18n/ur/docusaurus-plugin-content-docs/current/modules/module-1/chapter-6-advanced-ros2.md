---
title: "چیپٹر 6: اعلی ROS 2 تصورات"
sidebar_label: "چیپٹر 6: اعلی ROS 2 تصورات"
---

# چیپٹر 6: اعلی ROS 2 تصورات

## جائزہ

یہ چیپٹر اعلی ROS 2 تصورات کو تلاش کرتا ہے جو ترقی یافتہ روبوٹک ایپلی کیشنز تیار کرنے کے لیے ضروری ہیں۔ ہم لائف سائیکل نوڈس کو مضبوط سسٹم مینجمنٹ کے لیے، کمپوزیشن کو کارکردگی کے نوڈ آرگنائزیشن کے لیے، کارکردگی کی بہتری کے طریقے، اور سیکورٹی کے جاتے کو کور کریں گے۔ یہ اعلی درجے کے موضوعات حقیقی دنیا کے ماحول میں قابل اعتماد طریقے سے کام کرنے والے پیداواری سسٹم تیار کرنے کے لیے انتہائی اہم ہیں۔

اعلی ROS 2 خصوصیات ڈیولپرز کو زیادہ مضبوط، کارآمد، اور محفوظ روبوٹک ایپلی کیشنز تیار کرنے کے قابل بناتی ہیں۔ ان تصورات کو سمجھنا خاص طور پر ہیومنوائڈ روبوٹکس کے لیے اہم ہے جہاں قابل اعتمادی اور حفاظت سب سے اہم ہے۔ یہ چیپٹر پچھلے چیپٹرز سے بنیادی علم پر تعمیر کرتا ہے اور پیشہ ورانہ روبوٹک سسٹم میں استعمال ہونے والے نمونوں کو متعارف کراتا ہے۔

## سیکھنے کے اہداف

- مضبوط سسٹم کی حالت کے انتظام کے لیے لائف سائیکل نوڈس نافذ کریں
- کارآمد نوڈ معماری تیار کرنے کے لیے کمپوزیشن کا استعمال کریں
- حقیقی وقت کی ایپلی کیشنز کے لیے کارکردگی کی بہتری کے طریقے لاگو کریں
- ROS 2 ایپلی کیشنز کے لیے سیکورٹی اقدامات نافذ کریں
- DDS کنفیگریشن اور مڈل ویئر کے انتخاب کو سمجھیں
- اعلی درجے کے ڈیبگ اور پروفائلنگ کے طریقے لاگو کریں

## کلیدی تصورات

### لائف سائیکل نوڈس

لائف سائیکل نوڈس نوڈ کی ابتدا، کنفیگریشن، فعالیت، اور صفائی کے انتظام کے لیے ایک معیاری اسٹیٹ مشین فراہم کرتے ہیں۔ یہ زیادہ مضبوط سسٹم مینجمنٹ اور منسق کردہ اسٹارٹ اپ/شٹ ڈاؤن کی طریقہ کار کو فعال کرتا ہے۔

### نوڈ کمپوزیشن

نوڈ کمپوزیشن متعدد نوڈس کو ایک ہی عمل میں چلنے کی اجازت دیتا ہے، جس سے مواصلت کا اوور ہیڈ کم ہوتا ہے اور مضبوطی سے جڑے ہوئے اجزاء کے لیے کارکردگی میں بہتری آتی ہے۔

### سروس کی معیار (QoS) کی بہتری

اعلی QoS کنفیگریشن مخصوص کارکردگی اور قابل اعتمادی کی ضروریات کے لیے مواصلت کے رویے کو باریکی سے ایڈجسٹ کرنے کی اجازت دیتا ہے۔

### سیکورٹی فریم ورک

ROS 2 میں سیکورٹی کی صلاحیتیں شامل ہیں بشمول تصدیق، اجازت، اور خفیہ کاری تاکہ روبوٹک سسٹم کو غیر مجاز رسائی سے بچایا جا سکے۔

## تکنیکی گہرائی

### لائف سائیکل نوڈس

لائف سائیکل نوڈس ایک اسٹیٹ مشین نافذ کرتے ہیں جو مختلف آپریشنل حالت کے درمیان معیاری ٹرانزیشنز فراہم کرتا ہے:

**حالتیں:**
- غیر کنفیگر شدہ: نوڈ لوڈ کیا گیا لیکن کنفیگر نہیں کیا گیا
- غیر فعال: نوڈ کنفیگر کیا گیا لیکن فعال نہیں ہے
- فعال: نوڈ چل رہا ہے اور ڈیٹا کو پروسیس کر رہا ہے
- حتمی: نوڈ صاف کیا گیا اور تباہ کرنے کے لیے تیار ہے

**ٹرانزیشنز:**
- کنفیگر: غیر کنفیگر سے غیر فعال پر منتقلی
- کلین اپ: غیر فعال سے غیر کنفیگر پر منتقلی
- فعال کریں: غیر فعال سے فعال پر منتقلی
- غیر فعال کریں: فعال سے غیر فعال پر منتقلی
- شٹ ڈاؤن: حتمی حالت پر منتقلی

### نوڈ کمپوزیشن

نوڈ کمپوزیشن متعدد نوڈس کو ایک ہی عمل میں چلنے کی اجازت دیتا ہے، جس سے:
- بین الاقوامی عمل کی مواصلت کا اوور ہیڈ کم ہوتا ہے
- مضبوطی سے جڑے ہوئے نوڈس کے لیے کارکردگی میں بہتری آتی ہے
- ڈیپلومنٹ اور ریسورس مینجمنٹ کو آسان بناتا ہے
- ROS 2 کے تقسیم شدہ معماری کے فوائد برقرار رکھتا ہے

### DDS مڈل ویئر کنفیگریشن

DDS (ڈیٹا ڈسٹری بیوشن سروس) کنفیگریشن کا اثر ہوتا ہے:
- مواصلت کی کارکردگی اور قابل اعتمادی پر
- نیٹ ورک دریافت اور کنکشن قائم کرنے پر
- میموری اور CPU استعمال پر
- حقیقی وقت کا رویہ اور یقینیت پر

### اعلی درجے کی معماری (متن ڈائیگرام)

```
+-----------------------------------------------------------+
|                    اعلی ROS 2 سسٹم                      |
|                                                           |
|  +----------------+    +----------------+    +----------+ |
|  | لائف سائیکل نوڈ|    | کمپوز نوڈس     |    | سیکورٹی  | |
|  | (حالت مینجمنٹ) |    | (پروسیس گروپ)  |    | (DDS)    | |
|  | - غیر کنفیگر   |    | - نوڈ A       |    | - تصدیق  | |
|  | - غیر فعال     |    | - نوڈ B       |    | - کرپٹو  | |
|  | - فعال         |    | - نوڈ C       |    | - رسائی  | |
|  | - حتمی         |    +----------------+    | کنٹرول  | |
|  +----------------+                          +----------+ |
|         |                                        |        |
|         v                                        v        |
|  +----------------+                     +----------------+ |
|  | اسٹیٹ مینجر   |                     | سیکورٹی        | |
|  | (لائف سائیکل) |                     | مینجر         | |
|  | - م coordination|                     | - سرٹیفکیٹس  | |
|  | - مانیٹرنگ   |                     | - پالیسیز    | |
|  | - ریکوری     |                     | - نفاذ       | |
|  +----------------+                     +----------------+ |
|         |                                        |        |
|         +------------------سسٹم----------------+        |
|                           |                              |
|                    +------v------+                       |
|                    | کارکردگی کی    |                       |
|                    | بہتری         |                       |
|                    | - QoS ٹیوننگ  |                       |
|                    | - پروفائلنگ  |                       |
|                    | - مانیٹرنگ   |                       |
|                    +--------------+                       |
+-----------------------------------------------------------+
```

## کوڈ کی مثالیں

### لائف سائیکل نوڈ نافذ کرنا

```python
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from std_msgs.msg import String


class LifecycleTalker(LifecycleNode):
    """اسٹیٹ ٹرانزیشنز کو ظاہر کرنے والی مثال لائف سائیکل نوڈ"""

    def __init__(self, name):
        super().__init__(name)
        self.pub = None
        self.timer = None
        self.count = 0

        self.get_logger().info('لائف سائیکل ٹاکر نوڈ تیار کیا گیا، موجودہ حالت: غیر کنفیگر شدہ')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """نوڈ کو کنفیگر کرنے کے لیے کال بیک"""
        self.get_logger().info(f'نوڈ کو کنفیگر کر رہا ہے، پچھلی حالت: {state.label}')

        # کنفیگریشن کے دوران شائع کنندہ تیار کریں
        self.pub = self.create_publisher(String, 'lifecycle_chatter', 10)

        # ٹائمر تیار کریں (لیکن ابھی شروع نہ کریں)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.timer.cancel()  # فعال ہونے تک ٹائمر شروع نہ کریں

        self.count = 0

        self.get_logger().info('نوڈ کنفیگر ہو گیا، غیر فعال حالت کی طرف منتقل ہو رہا ہے')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """نوڈ کو صاف کرنے کے لیے کال بیک"""
        self.get_logger().info(f'نوڈ کو صاف کر رہا ہے، پچھلی حالت: {state.label}')

        # شائع کنندہ تباہ کریں
        self.destroy_publisher(self.pub)
        self.pub = None

        # ٹائمر تباہ کریں
        self.destroy_timer(self.timer)
        self.timer = None

        self.get_logger().info('نوڈ صاف ہو گیا، غیر کنفیگر حالت کی طرف منتقل ہو رہا ہے')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """نوڈ کو فعال کرنے کے لیے کال بیک"""
        self.get_logger().info(f'نوڈ کو فعال کر رہا ہے، پچھلی حالت: {state.label}')

        # شائع کنندہ کو فعال کریں
        self.pub.on_activate()

        # ٹائمر شروع کریں
        self.timer.reset()

        self.get_logger().info('نوڈ فعال ہو گیا، فعال حالت کی طرف منتقل ہو رہا ہے')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """نوڈ کو غیر فعال کرنے کے لیے کال بیک"""
        self.get_logger().info(f'نوڈ کو غیر فعال کر رہا ہے، پچھلی حالت: {state.label}')

        # ٹائمر رکو دیں
        self.timer.cancel()

        # شائع کنندہ کو غیر فعال کریں
        self.pub.on_deactivate()

        self.get_logger().info('نوڈ غیر فعال ہو گیا، غیر فعال حالت کی طرف منتقل ہو رہا ہے')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """نوڈ کو بند کرنے کے لیے کال بیک"""
        self.get_logger().info(f'نوڈ کو بند کر رہا ہے، پچھلی حالت: {state.label}')

        # حتمی صفائی کریں
        if self.pub is not None:
            self.destroy_publisher(self.pub)
        if self.timer is not None:
            self.destroy_timer(self.timer)

        self.get_logger().info('نوڈ بند ہو گیا، حتمی حالت کی طرف منتقل ہو رہا ہے')
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """پیغامات شائع کرنے کے لیے ٹائمر کال بیک"""
        msg = String()
        msg.data = f'لائف سائیکل msg {self.count}'
        self.count += 1

        # صرف اس وقت شائع کریں جب شائع کنندہ فعال ہو
        if self.pub is not None and self.pub.is_activated:
            self.pub.publish(msg)
            self.get_logger().info(f'شائع کیا گیا: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    # لائف سائیکل نوڈ تیار کریں
    lifecycle_node = LifecycleTalker('lifecycle_talker')

    # ایک منیجڈ نوڈ ایکزیکیوٹر تیار کریں
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(lifecycle_node)

    try:
        # کال بیکس کو پروسیس کرنے کے لیے اسپن کریں
        executor.spin()
    except KeyboardInterrupt:
        lifecycle_node.get_logger().info('مداخلت، بند کر رہا ہے')
    finally:
        lifecycle_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### نوڈ کمپوزیشن کی مثال

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Int32


class DataProcessorNode(Node):
    """ان پٹ ڈیٹا کو پروسیس کرنے والا نوڈ"""

    def __init__(self):
        super().__init__('data_processor')

        # سبسکرائبر تیار کریں
        self.subscription = self.create_subscription(
            String,
            'raw_data',
            self.listener_callback,
            QoSProfile(depth=10))

        # پروسیس کردہ ڈیٹا کے لیے شائع کنندہ تیار کریں
        self.publisher = self.create_publisher(
            Int32,
            'processed_data',
            QoSProfile(depth=10))

        self.get_logger().info('ڈیٹا پروسیسر نوڈ انشائیلائز ہوا')

    def listener_callback(self, msg):
        """ان کمنگ سٹرنگ پیغام کو پروسیس کریں اور لمبائی شائع کریں"""
        processed_msg = Int32()
        processed_msg.data = len(msg.data)

        self.publisher.publish(processed_msg)
        self.get_logger().info(f'پروسیس کیا گیا: "{msg.data}" -> لمبائی: {processed_msg.data}')


class DataGeneratorNode(Node):
    """ٹیسٹ ڈیٹا تیار کرنے والا نوڈ"""

    def __init__(self):
        super().__init__('data_generator')

        # شائع کنندہ تیار کریں
        self.publisher = self.create_publisher(
            String,
            'raw_data',
            QoSProfile(depth=10))

        # ڈیٹا تیار کرنے کے لیے ٹائمر تیار کریں
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0

        self.get_logger().info('ڈیٹا جنریٹر نوڈ انشائیلائز ہوا')

    def timer_callback(self):
        """ٹیسٹ ڈیٹا تیار کریں اور شائع کریں"""
        msg = String()
        msg.data = f'ٹیسٹ ڈیٹا پیغام #{self.counter}'
        self.counter += 1

        self.publisher.publish(msg)
        self.get_logger().info(f'شائع کیا گیا: {msg.data}')


def main(args=None):
    """نوڈ کمپوزیشن کو ظاہر کرنے والی مرکزی فنکشن"""
    rclpy.init(args=args)

    # ایک ہی عمل میں متعدد نوڈس تیار کریں
    processor_node = DataProcessorNode()
    generator_node = DataGeneratorNode()

    # ایکزیکیوٹر تیار کریں اور دونوں نوڈس شامل کریں
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(processor_node)
    executor.add_node(generator_node)

    try:
        # دونوں نوڈس ایک ہی عمل میں چل رہے ہیں
        executor.spin()
    except KeyboardInterrupt:
        processor_node.get_logger().info('مداخلت')
    finally:
        processor_node.destroy_node()
        generator_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### QoS کنفیگریشن کی مثال

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class QoSDemoNode(Node):
    """ مختلف QoS کنفیگریشنز کو ظاہر کرنے والا نوڈ"""

    def __init__(self):
        super().__init__('qos_demo_node')

        # زیادہ فریکوئنسی سینسر ڈیٹا (بہترین کوشش، متغیر)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sensor_publisher = self.create_publisher(
            String, 'sensor_data', sensor_qos)

        # اہم کنٹرول کمانڈز (قابل اعتماد، ٹرانزینٹ-مقامی)
        control_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.control_publisher = self.create_publisher(
            String, 'control_commands', control_qos)

        # کنفیگریشن پیرامیٹر (قابل اعتماد، مستقل)
        config_qos = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.config_publisher = self.create_publisher(
            String, 'configuration', config_qos)

        # مختلف QoS کے ساتھ شائع کرنے کے لیے ٹائمر تیار کریں
        self.timer = self.create_timer(1.0, self.publish_data)
        self.counter = 0

        self.get_logger().info(' مختلف پروفائلز کے ساتھ QoS ڈیمو نوڈ انشائیلائز ہوا')

    def publish_data(self):
        """ مختلف QoS پروفائلز کے ساتھ ڈیٹا شائع کریں"""
        # سینسر ڈیٹا شائع کریں (بہترین کوشش)
        sensor_msg = String()
        sensor_msg.data = f'سینسر ریڈنگ #{self.counter}'
        self.sensor_publisher.publish(sensor_msg)

        # کنٹرول کمانڈ شائع کریں (قابل اعتماد)
        control_msg = String()
        control_msg.data = f'کنٹرول کمانڈ #{self.counter}'
        self.control_publisher.publish(control_msg)

        # کنفیگریشن شائع کریں (مستقل)
        config_msg = String()
        config_msg.data = f'کنفیگریشن اپ ڈیٹ #{self.counter}'
        self.config_publisher.publish(config_msg)

        self.get_logger().info(f' مختلف QoS پروفائلز کے ساتھ پیغامات شائع کیے گئے (شمار: {self.counter})')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = QoSDemoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('مداخلت')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## عام مسائل

- **لائف سائیکل نوڈ کی پیچیدگی**: لائف سائیکل مینجمنٹ شامل کرنا نوڈ کی پیچیدگی میں کافی اضافہ کر سکتا ہے؛ یقینی بنائیں کہ آپ کے استعمال کے کیس کے لیے ضروری ہے
- **کمپوزیشن کی حدود**: کمپوز نوڈس ایک ہی عمل کی جگہ کو شیئر کرتے ہیں، لہذا ایک نوڈ میں کریش تمام کمپوز نوڈس کو متاثر کرتا ہے
- **QoS عدم مطابقتیں**: غیر مطابقت پذیر QoS پالیسیز والے شائع کنندہ اور سبسکرائبر مناسب طریقے سے بات چیت نہیں کر سکتے
- **سیکورٹی اوور ہیڈ**: سیکورٹی خصوصیات کمپیو ٹیشنل اوور ہیڈ شامل کرتی ہیں جو حقیقی وقت کی کارکردگی کو متاثر کر سکتی ہے
- **DDS کنفیگریشن**: غلط DDS ترتیبات دریافت کے مسائل یا کارکردگی کے مسائل کا سبب بن سکتی ہیں

## چیک پوائنٹس / مینی ایکسائزز

1. روبوٹ کی ابتدا کے لیے ایک لائف سائیکل نوڈ نافذ کریں جو مناسب اسٹیٹ ٹرانزیشنز سے گزرے
2. سینسر فیوژن کے لیے ایک نوڈس کی کمپوزیشن تیار کریں جو ایک ساتھ کام کریں
3. اپنے روبوٹ سسٹم میں مختلف پیغام کی اقسام کے لیے مختلف QoS پروفائلز کنفیگر کریں
4. ایک متعدد روبوٹ سسٹم کے لیے سیکورٹی پالیسیز سیٹ اپ کریں
5. اپنے ROS 2 ایپلی کیشن کی کارکردگی کو پروفائل اور بہتر بنائیں

## حوالہ جات

- [ROS 2 لائف سائیکل نوڈس](https://design.ros2.org/articles/node_lifecycle.html)
- [ROS 2 کمپوزیشن](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html)
- [ROS 2 سروس کی معیار](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-settings.html)
- [ROS 2 سیکورٹی](https://docs.ros.org/en/humble/Concepts/About-Security.html)