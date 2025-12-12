---
sidebar_position: 5
---

# باب 5: کیپ سٹون - مکمل خودکار ہیومنوڈ سسٹم

## جائزہ

یہ کیپ سٹون باب VLA (وژن-زبان-ایکشن) سسٹم کے تمام اجزاء کو ایک مکمل خودکار ہیومنوڈ روبوٹ میں ضم کرتا ہے جو ادراک، منطق، اور عمل کے قابل ہے۔ یہ باب یہ دکھاتا ہے کہ آواز پروسیسنگ، کوگنیٹو منصوبہ بندی، اور وژن سسٹم ایک متحدہ فریم ورک میں کس طرح ایک دوسرے کے ساتھ کام کرتے ہیں تاکہ پیچیدہ انسان-روبوٹ تعامل کو فعال کیا جا سکے۔ ہم مکمل VLA → ROS 2 → سیمولیشن انجام دہی لوپ کو نافذ کریں گے اور یہ دکھائیں گے کہ روبوٹ کس طرح مربوط ادراک، منطق، اور ایکشن کے ساتھ پیچیدہ آواز کے حکم کا جواب دے سکتا ہے۔

## سیکھنے کے اہداف

اس باب کے اختتام تک آپ درج ذیل کر سکیں گے:
- تمام VLA سسٹم کے اجزاء کو ایک متحدہ خودکار سسٹم میں ضم کریں
- مناسب خامی کے انتظام کے ساتھ مکمل VLA انجام دہی لوپ نافذ کریں
- آواز، وژن، اور ایکشن سسٹم کے درمیان ملٹی-موڈل کوآرڈینیشن ڈیزائن کریں
- خودکار آپریشن کے لیے خامی کی بازیافت اور ایڈاپٹیشن میکانزم نافذ کریں
- حقیقی وقت کی روبوٹک ایپلی کیشنز کے لیے سسٹم کارکردگی کو بہتر بنائیں
- پیچیدہ ٹاسک کے منظار کے خلاف مکمل سسٹم کی توثیق کریں
- ضم شدہ VLA سسٹم کے مسائل کو ڈیبگ اور ٹربولش کریں
- سسٹم کارکردگی کا جائزہ لیں اور بہتری کے مواقع کی شناخت کریں

## کلیدی تصورات

### VLA سسٹم انضمام
وژن، زبان، اور ایکشن اجزاء کا مکمل انضمام:
- ادراک، منطق، اور ایکشن کے درمیان حقیقی وقت کا کوآرڈینیشن
- تمام سسٹم کمپوننٹس کے مابین اسٹیٹ مینجمنٹ
- مسلسل ایڈاپٹیشن کے لیے فیڈ بیک لوپس
- پورے سسٹم میں سیفٹی میکانزمز

### ملٹی-موڈل کوآرڈینیشن
متعدد حسی اور ایکشن موڈلز کو مربوط کرنا:
- آواز، وژن، اور ایکشن کا ٹائمپل سنکرونائزیشن
- کراس-موڈل اٹینشن اور معلومات کا اشتراک
- مختلف موڈلز کے درمیان تنازعات کا حل
- موڈلز کے درمیان مسلسل اسٹیٹ کی نمائندگی

### خامی کی بازیافت اور ایڈاپٹیشن
ناکامیوں اور ماحولیاتی تبدیلیوں کو ہینڈل کرنے کے لیے مضبوط میکانزمز:
- ناکامی کا پتہ لگانا اور درجہ بندی
- گریس فل ڈیگریڈیشن کی حکمت عملیاں
- ماحولیاتی تبدیلیوں کے جواب میں منصوبہ دوبارہ منصوبہ بندی
- انسان-ان-دی-لوپ بازیافت میکانزمز

### کارکردگی کی بہتری
حقیقی وقت کے آپریشن کے لیے مکمل سسٹم کو بہتر بنانا:
- کمپیوٹیشنل وسائل کا مینجمنٹ
- کم سے کم لیٹنسی کے لیے پائپ لائن کی بہتری
- اسٹیٹ مینجمنٹ کے لیے موثر ڈیٹا سٹرکچر
- حقیقی وقت کی شیڈولنگ کے مسائل

## تکنیکی گہرائی

### مکمل VLA سسٹم آرکیٹیکچر

ضمن VLA سسٹم اس آرکیٹیکچر کو فالو کرتا ہے:

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                    انسانی حکم ان پٹ                            │
                    │                  (آواز، قدرتی زبان)                          │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────▼───────────────────────────────────┐
                    │                    VLA کوآرڈینیٹر                            │
                    │              (اسٹیٹ مینجمنٹ اور شیڈولنگ)                   │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────┼───────────────────────────────────┐
                    │                         │                                   │
        ┌───────────▼──────────┐    ┌────────▼────────┐    ┌─────────────────────▼─────────────┐
        │    آواز پروسیسر      │    │   کوگنیٹو        │    │      وژن                           │
        │   (Whisper + NLP)    │    │   منصوبہ ساز     │    │   پروسیسر                         │
        │                      │    │   (LLM-مبنی)     │    │   (آبجیکٹ ڈیکشن،                │
        │ • اسپیچ ریکوگنیشن  │    │ • ٹاسک            │    │    پوز اسٹیمیشن،                │
        │ • منشاء انخلا       │    │   ڈیکومپوزیشن   │    │    منظر کی سمجھ)                 │
        │ • کمپلیمنس اسکورنگ │    │ • منصوبہ بندی     │    │ • حقیقی وقت کی پروسیسنگ          │
        │ • نوائز فلٹرنگ     │    │   گرافس          │    │ • 3D پوزیشن اسٹیمیشن            │
        └──────────────────────┘    │ • رکاوٹ           │    └─────────────────────────────────┘
                                  │   ہینڈلنگ        │
                                  └───────────────┘
                                          │
                                          ▼
                            ┌─────────────────────────────────┐
                            │      ایکشن انجام دہی           │
                            │   (ROS 2 نیوی گیشن اور کنٹرول)  │
                            │                                 │
                            │ • نیوی گیشن ایکشنز            │
                            │ • مینیپولیشن ایکشنز           │
                            │ • ادراک ایکشنز                │
                            │ • مواصلت ایکشنز               │
                            └─────────────────────────────────┘
                                          │
                                          ▼
                            ┌─────────────────────────────────┐
                            │      فیڈ بیک اور مانیٹرنگ       │
                            │   (انجام کا اسٹیٹس اور بازیافت) │
                            │                                 │
                            │ • پیشرفت کا ٹریکنگ             │
                            │ • خامی کا پتہ لگانا             │
                            │ • منصوبہ ایڈاپٹیشن             │
                            │ • سیفٹی مانیٹرنگ               │
                            └─────────────────────────────────┘
```

### مکمل VLA سسٹم نفاذ

مرکزی نفاذ تمام VLA اجزاء کو ضم کرتا ہے:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, AudioData
from geometry_msgs.msg import Pose
import json
import threading
import queue
from typing import Dict, Any, Optional
from dataclasses import dataclass
import time
import uuid


@dataclass
class SystemState:
    """VLA سسٹم کی مکمل حالت کی نمائندگی کرتا ہے۔"""
    voice_commands: queue.Queue
    vision_detections: queue.Queue
    environment_state: Dict[str, Any]
    active_plan: Optional[Dict[str, Any]]
    robot_pose: Optional[Pose]
    system_status: str
    last_command_time: float
    error_recovery_mode: bool


class CompleteVLASystem(Node):
    """
    ہیومنوڈ روبوٹ کے خودکار آپریشن کے لیے آواز، وژن، اور ایکشن اجزاء کو ضم کرنے والا مکمل VLA سسٹم۔
    """

    def __init__(self):
        super().__init__('complete_vla_system')

        # سسٹم اسٹیٹ شروع کریں
        self.system_state = SystemState(
            voice_commands=queue.Queue(),
            vision_detections=queue.Queue(),
            environment_state={},
            active_plan=None,
            robot_pose=None,
            system_status="idle",
            last_command_time=0.0,
            error_recovery_mode=False
        )

        # پبلشرز
        self.command_publisher = self.create_publisher(
            String,
            'robot/command',
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            'vla_system/status',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            'vla_system/feedback',
            10
        )

        # سبسکرائبرز
        self.voice_subscriber = self.create_subscription(
            String,
            'voice_command',
            self.voice_callback,
            10
        )

        self.vision_subscriber = self.create_subscription(
            String,
            'vision/detections',
            self.vision_callback,
            10
        )

        self.environment_subscriber = self.create_subscription(
            String,
            'environment/state',
            self.environment_callback,
            10
        )

        self.robot_pose_subscriber = self.create_subscription(
            Pose,
            'robot/pose',
            self.robot_pose_callback,
            10
        )

        # کنٹرول سبسکرائبرز
        self.emergency_stop_subscriber = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10
        )

        # سسٹم کنٹرول
        self.main_control_thread = None
        self.shutdown_requested = False

        # پیرامیٹرز
        self.declare_parameter('max_command_age', 30.0)  # سیکنڈ
        self.declare_parameter('error_recovery_enabled', True)
        self.declare_parameter('system_heartbeat_interval', 1.0)  # سیکنڈ
        self.declare_parameter('execution_timeout', 300.0)  # سیکنڈ

        # مرکزی کنٹرول تھریڈ شروع کریں
        self.main_control_thread = threading.Thread(target=self.main_control_loop, daemon=True)
        self.main_control_thread.start()

        # ہارٹ بیٹ پبلشر شروع کریں
        self.heartbeat_timer = self.create_timer(
            self.get_parameter('system_heartbeat_interval').value,
            self.publish_system_heartbeat
        )

        self.get_logger().info('مکمل VLA سسٹم شروع کیا گیا')

    def voice_callback(self, msg):
        """
        آنے والے آواز کے حکم کو ہینڈل کریں۔
        """
        try:
            command_data = json.loads(msg.data)
            self.system_state.voice_commands.put(command_data)
            self.system_state.last_command_time = time.time()

            self.get_logger().info(f'آواز کا حکم موصول ہوا: {command_data.get("command", "unknown")}')

            # سسٹم اسٹیٹس اپ ڈیٹ کریں
            self.system_state.system_status = "processing_command"
            self.publish_system_status()

        except json.JSONDecodeError:
            self.get_logger().error('غلط آواز کمانڈ JSON')

    def vision_callback(self, msg):
        """
        آنے والی وژن ڈیٹکشنز کو ہینڈل کریں۔
        """
        try:
            detection_data = json.loads(msg.data)
            self.system_state.vision_detections.put(detection_data)

            # وژن معلومات کے ساتھ ماحولیاتی حالت کو اپ ڈیٹ کریں
            self.update_environment_with_vision(detection_data)

            self.get_logger().debug(f'وژن ڈیکشن موصول ہوا: {len(detection_data.get("detections", []))} اشیاء')

        except json.JSONDecodeError:
            self.get_logger().error('غلط وژن ڈیٹا JSON')

    def environment_callback(self, msg):
        """
        ماحولیاتی حالت کے اپ ڈیٹس کو ہینڈل کریں۔
        """
        try:
            env_data = json.loads(msg.data)
            self.system_state.environment_state.update(env_data)
        except json.JSONDecodeError:
            self.get_logger().error('غلط ماحولیاتی حالت JSON')

    def robot_pose_callback(self, msg):
        """
        روبوٹ پوز اپ ڈیٹس کو ہینڈل کریں۔
        """
        self.system_state.robot_pose = msg

    def emergency_stop_callback(self, msg):
        """
        ایمرجنسی اسٹاپ کمانڈز کو ہینڈل کریں۔
        """
        if msg.data:
            self.get_logger().warn('ایمرجنسی اسٹاپ فعال ہوا!')
            self.system_state.system_status = "emergency_stop"
            self.cancel_active_plan()
            self.publish_system_status()

    def main_control_loop(self):
        """
        VLA سسٹم کے لیے مرکزی کنٹرول لوپ۔
        """
        while not self.shutdown_requested:
            try:
                # آواز کے حکم کو پروسیس کریں
                self.process_voice_commands()

                # فعال منصوبے کو مانیٹر کریں
                self.monitor_active_plan()

                # سسٹم کی خامیوں کی چیک کریں
                self.check_system_errors()

                # اگر ضرورت ہو تو خامی کی بازیافت ہینڈل کریں
                if self.system_state.error_recovery_mode:
                    self.handle_error_recovery()

                # مصروف انتظار سے بچنے کے لیے چھوٹی سلیپ
                time.sleep(0.01)

            except Exception as e:
                self.get_logger().error(f'مرکزی کنٹرول لوپ میں خامی: {e}')
                time.sleep(0.1)  # خامی پر مختصر وقفہ

    def process_voice_commands(self):
        """
        قطار میں رکھے گئے آواز کے حکم کو پروسیس کریں اور منصوبے تیار کریں۔
        """
        while not self.system_state.voice_commands.empty():
            try:
                command_data = self.system_state.voice_commands.get_nowait()

                # حکم اور ماحول کی بنیاد پر منصوبہ تیار کریں
                plan = self.generate_plan_from_command(command_data)

                if plan:
                    # منصوبہ انجام دیں
                    self.execute_plan(plan)
                else:
                    self.get_logger().error('حکم سے منصوبہ تیار کرنے میں ناکامی')

            except queue.Empty:
                break
            except Exception as e:
                self.get_logger().error(f'آواز کمانڈ کی پروسیسنگ میں خامی: {e}')

    def generate_plan_from_command(self, command_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        کوگنیٹو منصوبہ بندی کا استعمال کرتے ہوئے آواز کے حکم سے ایک منصوبہ تیار کریں۔
        """
        try:
            command_text = command_data.get('command', '')
            if not command_text:
                return None

            # اس مثال کے لیے، ہم ایک سادہ منصوبہ تیار کریں گے
            # عمل میں، یہ LLM-مبنی منصوبہ ساز کا استعمال کرے گا
            plan_id = str(uuid.uuid4())

            # حکم کی بنیاد پر ایک سادہ منصوبہ تیار کریں
            if "bring me" in command_text.lower() or "get me" in command_text.lower():
                plan = {
                    "id": plan_id,
                    "original_command": command_text,
                    "tasks": [
                        {
                            "id": f"locate_{plan_id}",
                            "type": "perception",
                            "action": "detect_object",
                            "parameters": {"object_type": "requested_item"},
                            "dependencies": []
                        },
                        {
                            "id": f"navigate_{plan_id}",
                            "type": "navigation",
                            "action": "move_to_location",
                            "parameters": {"x": 1.0, "y": 1.0, "theta": 0.0},
                            "dependencies": [f"locate_{plan_id}"]
                        },
                        {
                            "id": f"grasp_{plan_id}",
                            "type": "manipulation",
                            "action": "grasp_object",
                            "parameters": {"object_type": "requested_item"},
                            "dependencies": [f"navigate_{plan_id}"]
                        },
                        {
                            "id": f"return_{plan_id}",
                            "type": "navigation",
                            "action": "return_to_user",
                            "parameters": {},
                            "dependencies": [f"grasp_{plan_id}"]
                        }
                    ],
                    "estimated_duration": 120.0
                }
            elif "clean" in command_text.lower():
                plan = {
                    "id": plan_id,
                    "original_command": command_text,
                    "tasks": [
                        {
                            "id": f"scan_{plan_id}",
                            "type": "perception",
                            "action": "scan_environment",
                            "parameters": {},
                            "dependencies": []
                        },
                        {
                            "id": f"locate_trash_{plan_id}",
                            "type": "perception",
                            "action": "detect_trash",
                            "parameters": {},
                            "dependencies": [f"scan_{plan_id}"]
                        },
                        {
                            "id": f"pickup_trash_{plan_id}",
                            "type": "manipulation",
                            "action": "pickup_trash",
                            "parameters": {"object_id": "trash_1"},
                            "dependencies": [f"locate_trash_{plan_id}"]
                        },
                        {
                            "id": f"dispose_trash_{plan_id}",
                            "type": "manipulation",
                            "action": "dispose_trash",
                            "parameters": {"object_id": "trash_1"},
                            "dependencies": [f"pickup_trash_{plan_id}"]
                        }
                    ],
                    "estimated_duration": 300.0
                }
            else:
                # غیر پہچانے گئے حکم کے لیے ڈیفالٹ منصوبہ
                plan = {
                    "id": plan_id,
                    "original_command": command_text,
                    "tasks": [
                        {
                            "id": f"request_clarification_{plan_id}",
                            "type": "communication",
                            "action": "request_clarification",
                            "parameters": {"message": f"میں یہ سمجھ نہیں سکتا: {command_text}"},
                            "dependencies": []
                        }
                    ],
                    "estimated_duration": 10.0
                }

            return plan

        except Exception as e:
            self.get_logger().error(f'منصوبہ تیار کرنے میں خامی: {e}')
            return None

    def execute_plan(self, plan: Dict[str, Any]):
        """
        ایک تیار کردہ منصوبہ انجام دیں۔
        """
        self.get_logger().info(f'منصوبہ انجام دیا جا رہا ہے: {plan["id"]}')

        self.system_state.active_plan = plan
        self.system_state.system_status = "executing_plan"
        self.publish_system_status()

        # حقیقی نفاذ میں، یہ منصوبہ میں کام انجام دے گا
        # اس مثال کے لیے، ہم انجام دہی کو محاکمہ کریں گے
        self.simulate_plan_execution(plan)

    def simulate_plan_execution(self, plan: Dict[str, Any]):
        """
        مظاہرے کے مقاصد کے لیے منصوبہ انجام دہی کو محاکمہ کریں۔
        """
        try:
            # منصوبہ میں ہر کام کو شائع کریں
            for task in plan.get('tasks', []):
                if self.system_state.error_recovery_mode:
                    self.get_logger().info('خامی کی بازیافت کی وجہ سے منصوبہ انجام دہی روک دی گئی')
                    break

                # انجام دہی کے لیے کام شائع کریں
                task_msg = String()
                task_msg.data = json.dumps({
                    'plan_id': plan['id'],
                    'task': task,
                    'environment': self.system_state.environment_state
                })
                self.command_publisher.publish(task_msg)

                self.get_logger().info(f'کام شائع کیا گیا: {task["id"]}')

                # کام کی انجام دہی کا وقت محاکمہ کریں
                time.sleep(0.5)

            # منصوبہ مکمل ہونے کا نشان لگائیں
            self.system_state.active_plan = None
            self.system_state.system_status = "idle"
            self.publish_system_status()

            feedback_msg = String()
            feedback_msg.data = f'منصوبہ {plan["id"]} کامیابی سے مکمل ہوا'
            self.feedback_publisher.publish(feedback_msg)

        except Exception as e:
            self.get_logger().error(f'منصوبہ انجام دہی میں خامی: {e}')
            self.handle_execution_error(e)

    def monitor_active_plan(self):
        """
        مکمل ہونے یا خامیوں کے لیے فعال منصوبہ کو مانیٹر کریں۔
        """
        if self.system_state.active_plan is None:
            return

        # چیک کریں کہ کیا منصوبہ کا ٹائم آؤٹ ختم ہو گیا ہے
        if (time.time() - self.system_state.last_command_time) > self.get_parameter('execution_timeout').value:
            self.get_logger().warn('فعال منصوبہ کا ٹائم آؤٹ ختم ہو گیا، منسوخ کیا جا رہا ہے')
            self.cancel_active_plan()
            self.system_state.error_recovery_mode = True

    def cancel_active_plan(self):
        """
        فی الحال فعال منصوبہ منسوخ کریں۔
        """
        if self.system_state.active_plan:
            self.get_logger().info(f'منصوبہ منسوخ کیا جا رہا ہے: {self.system_state.active_plan["id"]}')
            self.system_state.active_plan = None

        self.system_state.system_status = "idle"
        self.publish_system_status()

    def check_system_errors(self):
        """
        مختلف سسٹم کی خامیوں کی چیک کریں۔
        """
        # پرانے حکم کی چیک کریں
        if (time.time() - self.system_state.last_command_time) > self.get_parameter('max_command_age').value:
            if self.system_state.system_status == "processing_command":
                self.get_logger().warn('حکم بہت پرانا ہو گیا، سسٹم اسٹیٹ ری سیٹ کیا جا رہا ہے')
                self.system_state.system_status = "idle"
                self.publish_system_status()

    def handle_error_recovery(self):
        """
        سسٹم کی خامی کی بازیافت ہینڈل کریں۔
        """
        if not self.get_parameter('error_recovery_enabled').value:
            return

        self.get_logger().info('خامی کی بازیافت موڈ میں داخل ہو رہا ہے')

        # تمام موجودہ سرگرمیاں بند کریں
        self.cancel_active_plan()

        # کمانڈ قطار صاف کریں
        while not self.system_state.voice_commands.empty():
            try:
                self.system_state.voice_commands.get_nowait()
            except queue.Empty:
                break

        # بازیافت اسٹیٹس شائع کریں
        feedback_msg = String()
        feedback_msg.data = 'سسٹم خامی کی بازیافت موڈ میں ہے، نئے حکم کا انتظار کر رہا ہے'
        self.feedback_publisher.publish(feedback_msg)

        # ایک تاخیر کے بعد خامی کی بازیافت موڈ ری سیٹ کریں
        time.sleep(2.0)
        self.system_state.error_recovery_mode = False
        self.get_logger().info('خامی کی بازیافت مکمل، نارمل آپریشن پر واپسی')

    def handle_execution_error(self, error: Exception):
        """
        منصوبہ انجام دہی کے دوران خامیوں کو ہینڈل کریں۔
        """
        self.get_logger().error(f'انجام دہی کی خامی: {error}')

        # موجودہ منصوبہ منسوخ کریں
        self.cancel_active_plan()

        # خامی کی بازیافت موڈ میں داخل ہوں
        self.system_state.error_recovery_mode = True

        # خامی کا فیڈ بیک شائع کریں
        feedback_msg = String()
        feedback_msg.data = f'انجام دہی کی خامی: {str(error)}, بازیافت موڈ میں داخل ہو رہا ہے'
        self.feedback_publisher.publish(feedback_msg)

    def update_environment_with_vision(self, detection_data: Dict[str, Any]):
        """
        وژن معلومات کے ساتھ ماحولیاتی حالت کو اپ ڈیٹ کریں۔
        """
        # اشیاء کے مقامات اور منظر کی معلومات کے ساتھ ماحول کو اپ ڈیٹ کریں
        if 'detections' in detection_data:
            self.system_state.environment_state['objects'] = detection_data['detections']

        if 'scene' in detection_data:
            self.system_state.environment_state['scene'] = detection_data['scene']

    def publish_system_status(self):
        """
        موجودہ سسٹم اسٹیٹس شائع کریں۔
        """
        status_msg = String()
        status_msg.data = json.dumps({
            'status': self.system_state.system_status,
            'active_plan_id': self.system_state.active_plan['id'] if self.system_state.active_plan else None,
            'error_recovery_mode': self.system_state.error_recovery_mode,
            'timestamp': time.time()
        })
        self.status_publisher.publish(status_msg)

    def publish_system_heartbeat(self):
        """
        سسٹم کا ہارٹ بیٹ شائع کریں تاکہ سسٹم زندہ ہونے کی نشاندہی ہو۔
        """
        heartbeat_msg = String()
        heartbeat_msg.data = json.dumps({
            'system': 'complete_vla_system',
            'status': self.system_state.system_status,
            'timestamp': time.time(),
            'active_plan': self.system_state.active_plan is not None
        })
        self.status_publisher.publish(heartbeat_msg)

    def destroy_node(self):
        """
        نوڈ کو تباہ کرنے سے پہلے وسائل صاف کریں۔
        """
        self.shutdown_requested = True
        if self.main_control_thread:
            self.main_control_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """
    مکمل VLA سسٹم چلانے کے لیے مرکزی فنکشن۔
    """
    rclpy.init(args=args)

    vla_system = CompleteVLASystem()

    try:
        rclpy.spin(vla_system)
    except KeyboardInterrupt:
        vla_system.get_logger().info('صارف کی طرف سے مداخلت، بند کیا جا رہا ہے...')
    finally:
        vla_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## کوڈ کی مثالیں

### VLA سسٹم انضمام کی مثال

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
import json
import threading
import queue
from typing import Dict, Any, Optional
import time


class VLAIntegrationExample(Node):
    """
    پیچیدہ ٹاسک انجام دہی کے لیے VLA اجزاء کو ضم کرنے کی مثال۔
    """

    def __init__(self):
        super().__init__('vla_integration_example')

        # سسٹم کمپوننٹس کے لیے پبلشرز
        self.voice_command_publisher = self.create_publisher(
            String,
            'voice_command',
            10
        )

        self.vision_command_publisher = self.create_publisher(
            String,
            'vision/command',
            10
        )

        self.planning_command_publisher = self.create_publisher(
            String,
            'planning/command',
            10
        )

        self.robot_command_publisher = self.create_publisher(
            String,
            'robot/command',
            10
        )

        # سسٹم فیڈ بیک کے لیے سبسکرائبرز
        self.system_status_subscriber = self.create_subscription(
            String,
            'vla_system/status',
            self.system_status_callback,
            10
        )

        self.execution_feedback_subscriber = self.create_subscription(
            String,
            'vla_system/feedback',
            self.execution_feedback_callback,
            10
        )

        # انٹرنل اسٹیٹ
        self.system_status = {}
        self.active_demo = None
        self.demo_thread = None

        # پیرامیٹرز
        self.declare_parameter('demo_execution_delay', 2.0)

        self.get_logger().info('VLA انضمام کی مثال شروع کی گئی')

    def system_status_callback(self, msg):
        """
        سسٹم اسٹیٹس اپ ڈیٹس کو ہینڈل کریں۔
        """
        try:
            self.system_status = json.loads(msg.data)
            self.get_logger().debug(f'سسٹم اسٹیٹس اپ ڈیٹ ہوا: {self.system_status.get("status", "unknown")}')
        except json.JSONDecodeError:
            self.get_logger().error('غلط سسٹم اسٹیٹس JSON')

    def execution_feedback_callback(self, msg):
        """
        انجام دہی کا فیڈ بیک ہینڈل کریں۔
        """
        self.get_logger().info(f'انجام دہی کا فیڈ بیک: {msg.data}')

    def run_complex_demo(self, command: str):
        """
        VLA انضمام کی پیچیدہ مظاہرہ چلائیں۔
        """
        self.get_logger().info(f'کمانڈ کے ساتھ پیچیدہ ڈیمو شروع کر رہا ہے: {command}')

        # آواز کا حکم بھیجیں
        voice_msg = String()
        voice_msg.data = json.dumps({
            'command': command,
            'timestamp': time.time(),
            'source': 'integration_demo'
        })
        self.voice_command_publisher.publish(voice_msg)

        # سسٹم کے پروسیس کرنے کے لیے انتظار کریں
        time.sleep(self.get_parameter('demo_execution_delay').value)

        # پیشرفت کو مانیٹر کریں
        self.monitor_demo_progress()

    def monitor_demo_progress(self):
        """
        ڈیمو انجام دہی کی پیشرفت کو مانیٹر کریں۔
        """
        start_time = time.time()
        max_duration = 300.0  # 5 منٹ زیادہ سے زیادہ

        while time.time() - start_time < max_duration:
            if self.system_status.get('status') == 'idle':
                self.get_logger().info('ڈیمو کامیابی سے مکمل ہوا')
                break
            elif self.system_status.get('error_recovery_mode', False):
                self.get_logger().warn('ڈیمو خامی کی بازیافت موڈ میں داخل ہوا')
                break
            elif self.system_status.get('status') == 'emergency_stop':
                self.get_logger().warn('ایمرجنسی اسٹاپ کی وجہ سے ڈیمو بند کر دیا گیا')
                break

            time.sleep(1.0)

    def demonstrate_vla_capabilities(self):
        """
        مختلف حکم کے ساتھ مختلف VLA صلاحیتوں کا مظاہرہ کریں۔
        """
        demo_commands = [
            "میز سے مجھے لال کپ لے کر دیں",
            "کچرا اٹھا کر کمرہ صاف کریں",
            "کچن میں جائیں اور نیلا بوتل تلاش کریں",
            "کتابوں کو الماری پر رکھ کر ڈیسک کو ترتیب دیں"
        ]

        for i, command in enumerate(demo_commands):
            self.get_logger().info(f'ڈیمو {i+1}/{len(demo_commands)}: {command}')
            self.run_complex_demo(command)

            # ڈیمو کے درمیان انتظار کریں
            time.sleep(5.0)

        self.get_logger().info('تمام VLA صلاحیت کے مظاہرے مکمل ہو گئے')

    def run_autonomous_scenario(self):
        """
        مسلسل حکم کی پروسیسنگ کے ساتھ ایک خودکار منظر نامہ چلائیں۔
        """
        self.get_logger().info('خودکار منظر نامہ شروع کر رہا ہے')

        # مسلسل کمانڈ ان پٹ کا محاکمہ کریں
        commands = [
            "کمرہ کے ارد گرد دیکھیں",
            "کرسی تلاش کریں",
            "کرسی کی طرف جائیں",
            "کرسی کے قریب انتظار کریں"
        ]

        for command in commands:
            self.get_logger().info(f'خودکار کمانڈ پروسیس کر رہا ہے: {command}')

            # حکم بھیجیں
            voice_msg = String()
            voice_msg.data = json.dumps({
                'command': command,
                'timestamp': time.time(),
                'source': 'autonomous_scenario'
            })
            self.voice_command_publisher.publish(voice_msg)

            # اگلے حکم سے پہلے مکمل ہونے کا انتظار کریں
            time.sleep(10.0)

        self.get_logger().info('خودکار منظر نامہ مکمل ہوا')


def main(args=None):
    """
    VLA انضمام کی مثال چلانے کے لیے مرکزی فنکشن۔
    """
    rclpy.init(args=args)

    integration_example = VLAIntegrationExample()

    try:
        # مظاہرہ چلائیں
        integration_example.demonstrate_vla_capabilities()

        # ایک خودکار منظر نامہ چلائیں
        integration_example.run_autonomous_scenario()

        # ممکنہ دستی کمانڈز کے لیے نوڈ کو زندہ رکھیں
        rclpy.spin(integration_example)

    except KeyboardInterrupt:
        integration_example.get_logger().info('صارف کی طرف سے مداخلت، بند کیا جا رہا ہے...')
    finally:
        integration_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## ڈائریم (متن-مبنی)

### مکمل VLA انجام دہی لوپ

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                    شروع: صارف کا حکم                           │
                    │                    (آواز ان پٹ)                               │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────▼───────────────────────────────────┐
                    │                    آواز پروسیسنگ                            │
                    │                  (Whisper ٹرانسکرپشن)                       │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────▼───────────────────────────────────┐
                    │                   NLP پروسیسنگ                             │
                    │                (منشاء انخلا)                               │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────┼───────────────────────────────────┐
                    │                         │                                   │
        ┌───────────▼──────────┐    ┌────────▼────────┐    ┌─────────────────────▼─────────────┐
        │    ماحولیاتی         │    │   کوگنیٹو        │    │      وژن                           │
        │    حالت              │    │   منصوبہ بندی    │    │   پروسیسنگ                        │
        │   (موجودہ دنیا       │    │   (LLM ٹاسک      │    │   (آبجیکٹ ڈیکشن،                │
        │    ماڈل)             │    │    ڈیکومپوزیشن  │    │    منظر کی سمجھ)                  │
        │                      │    │    اور منصوبہ    │    │                                   │
        │ • اشیاء کے مقامات    │    │    بندی)         │    │ • حقیقی وقت کی پروسیسنگ          │
        │ • روبوٹ پوز          │    │ • کام            │    │ • 3D پوزیشن اسٹیمیشن            │
        │ • رکاوٹیں            │    │   ترتیبات       │    │ • جگہی تعلقات                   │
        └──────────────────────┘    │ • انحصار         │    └─────────────────────────────────┘
                                  │   ہینڈلنگ        │
                                  └───────────────┘
                                          │
                                          ▼
                            ┌─────────────────────────────────┐
                            │      منصوبہ انجام دہی           │
                            │   (ROS 2 ایکشن انجام دہی)       │
                            │                                 │
                            │ • نیوی گیشن ایکشنز            │
                            │ • مینیپولیشن ایکشنز           │
                            │ • ادراک ایکشنز                │
                            │ • مواصلت ایکشنز               │
                            └─────────────────────────────────┘
                                          │
                                          ▼
                            ┌─────────────────────────────────┐
                            │      انجام دہی مانیٹرنگ       │
                            │   (پیشرفت اور خامی ٹریکنگ)    │
                            │                                 │
                            │ • کام مکمل ہونے کا اسٹیٹس      │
                            │ • خامی کا پتہ لگانا             │
                            │ • منصوبہ ایڈاپٹیشن ٹریگرز     │
                            │ • سیفٹی مانیٹرنگ               │
                            └─────────────────────────────────┘
                                          │
                                          ▼
                            ┌─────────────────────────────────┐
                            │         فیڈ بیک لوپ             │
                            │   (سسٹم اسٹیٹس اور ایڈاپٹیشن)  │
                            │                                 │
                            │ • کامیابی/ناکامی کی رپورٹنگ     │
                            │ • منصوبہ کی تبدیلی             │
                            │ • بازیافت ایکشنز               │
                            │ • سیکھنے کے اپ ڈیٹس             │
                            └─────────────────────────────────┘
                                          │
                                          ▼
                            ┌─────────────────────────────────┐
                            │            دہرائیں              │
                            │      (نیا حکم ان پٹ)             │
                            └─────────────────────────────────┘
```

### ملٹی-موڈل کوآرڈینیشن آرکیٹیکچر

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   آواز ان پٹ    │    │  وژن ان پٹ      │    │  ایکشن آؤٹ پٹ  │
│                 │    │                 │    │                 │
│ • سپیچ          │    │ • RGB کیمرہ    │    │ • نیوی گیشن    │
│ • مائیکروفون    │    │ • ڈیپتھ سینسر  │    │ • مینیپولیشن   │
│ • آڈیو سٹریم    │    │ • متعدد         │    │ • مواصلت       │
└─────────┬───────┘    │   کیمرے        │    │ • ادراک        │
          │            └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          │            ┌─────────▼─────────┐            │
          │            │  VLA کوآرڈینیٹر  │            │
          │            │                   │            │
          └────────────► • اسٹیٹ مینجمنٹ  │◄───────────┘
                       │ • سنکرونائزیشن   │
                       │ • تنازعہ          │
                       │   حل              │
                       │ • ٹائم کنٹرول    │
                       └─────────┬─────────┘
                                 │
                    ┌────────────▼────────────┐
                    │   انضمام لیئر           │
                    │                         │
                    │ • کراس-موڈل اٹینشن   │
                    │ • ملٹی-موڈل فیوژن     │
                    │ • فیصلہ سازی          │
                    │ • انجام دہی شیڈولنگ  │
                    └─────────────────────────┘
```

## عام مسائل

### 1. سسٹم انضمام کی پیچیدگی
**مسئلہ**: متعدد پیچیدہ سسٹم کو ضم کرنا کوآرڈینیشن کے چیلنج پیدا کرتا ہے۔
**حل**: صاف انٹرفیسز نافذ کریں، میسج پاسنگ کا استعمال کریں، اور ماڈولریٹی کے لیے ڈیزائن کریں۔

### 2. حقیقی وقت کی کارکردگی کے مسائل
**مسئلہ**: مجموعی سسٹم حقیقی وقت کی ضروریات پوری نہیں کر سکتا۔
**حل**: ہر کمپوننٹ کو بہتر بنائیں، موثر الگوری دھم استعمال کریں، اور مناسب شیڈولنگ نافذ کریں۔

### 3. خامی کا پھیلاؤ
**مسئلہ**: ایک کمپوننٹ میں خامیاں سسٹم کے ذریعے پھیل سکتی ہیں۔
**حل**: خامی کا جزیرہ بندی نافذ کریں، گریس فل ڈیگریڈیشن، اور بازیافت کے میکانزمز نافذ کریں۔

### 4. اسٹیٹ کی مطابقت
**مسئلہ**: مختلف کمپوننٹس کے دنیا کے بارے میں غیر مطابق خیالات ہو سکتے ہیں۔
**حل**: مرکزی اسٹیٹ مینجمنٹ اور سنکرونائزیشن پروٹوکولز نافذ کریں۔

### 5. وسائل کی مقابلہ
**مسئلہ**: متعدد کمپوننٹس کمپیوٹیشنل وسائل کے لیے مقابلہ کر سکتے ہیں۔
**حل**: وسائل کی تفویض کی حکمت عملیاں اور ترجیح-مبنی شیڈولنگ نافذ کریں۔

## چیک پوائنٹس

### سمجھ کا چیک 1: سسٹم انضمام
- مکمل سسٹم میں آواز، وژن، اور ایکشن کمپوننٹس کس طرح کوآرڈینیٹ کرتے ہیں؟
- متعدد AI سسٹم کو ضم کرنے میں کیا چیلنج ہیں؟
- سسٹم کس طرح کمپوننٹس کے درمیان ٹائم اور سنکرونائزیشن کو ہینڈل کرتا ہے؟

### سمجھ کا چیک 2: خامی ہینڈلنگ
- سسٹم میں کون سے خامی کی بازیافت کے میکانزمز نافذ کیے گئے ہیں؟
- سسٹم جزوی ناکامیوں کو کس طرح ہینڈل کرتا ہے؟
- نقصان دہ روبوٹ ایکشنز کو روکنے کے لیے کون سے سیفٹی اقدامات ہیں؟

### اطلاق کا چیک: مکمل سسٹم
- آپ سسٹم کو ایک مخصوص روبوٹک پلیٹ فارم کے لیے کیسے تبدیل کریں گے؟
- کون سے اضافی سینسرز یا صلاحیتیں سسٹم کو بہتر کریں گی؟
- آپ سسٹم کو ایک مخصوص اطلاق کے ڈومین کے لیے کیسے بہتر بنائیں گے؟

## موڈیول خلاصہ ڈائریم

### مکمل VLA سسٹم آرکیٹیکچر

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                    صارف انٹرایکشن                           │
                    │                  (آواز کے حکم)                              │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────▼───────────────────────────────────┐
                    │                    VLA کوآرڈینیٹر                           │
                    │              (اسٹیٹ مینجمنٹ اور شیڈولنگ)                   │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────┼───────────────────────────────────┐
                    │                         │                                   │
        ┌───────────▼──────────┐    ┌────────▼────────┐    ┌─────────────────────▼─────────────┐
        │    آواز سسٹم         │    │  کوگنیٹو         │    │      وژن                           │
        │   (باب 2)            │    │  منصوبہ بندی     │    │   سسٹم                            │
        │   (Whisper)           │    │  (باب 3)        │    │   (باب 4)                         │
        │                      │    │  (LLM-مبنی)      │    │                                   │
        │ • اسپیچ ریکوگنیشن   │    │ • ٹاسک            │    │ • آبجیکٹ ڈیکشن                   │
        │ • منشاء انخلا        │    │   ڈیکومپوزیشن   │    │ • پوز اسٹیمیشن                   │
        │ • کمپلیمنس اسکورنگ  │    │ • منصوبہ بندی     │    │ • منظر کی سمجھ                     │
        │ • نوائز فلٹرنگ      │    │   گرافس          │    │ • جگہی منطق                       │
        └──────────────────────┘    │ • رکاوٹ           │    └─────────────────────────────────┘
                                  │   ہینڈلنگ        │
                                  └───────────────┘
                                          │
                                          ▼
                            ┌─────────────────────────────────┐
                            │      ایکشن انجام دہی           │
                            │   (ROS 2 انضمام - M1-3)        │
                            │                                 │
                            │ • نیوی گیشن (M1-3)             │
                            │ • مینیپولیشن (M1-3)            │
                            │ • ادراک ایکشنز (M1-3)         │
                            │ • مواصلت (M1-3)                │
                            └─────────────────────────────────┘
                                          │
                                          ▼
                            ┌─────────────────────────────────┐
                            │      فیڈ بیک اور مانیٹرنگ      │
                            │   (انجام دہی اسٹیٹس اور بازیافت) │
                            │                                 │
                            │ • پیشرفت ٹریکنگ                │
                            │ • خامی کا پتہ لگانا             │
                            │ • منصوبہ ایڈاپٹیشن             │
                            │ • سیفٹی مانیٹرنگ               │
                            └─────────────────────────────────┘
```

### پچھلے موڈیولز کے ساتھ موڈیول انضمام

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        مکمل ہیومنوڈ روبوٹ سسٹم                              │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │
│  │   موڈیول 1      │  │   موڈیول 2      │  │   موڈیول 3      │  │ موڈیول 4    │ │
│  │   (ROS 2)       │  │   (ڈیجیٹل ٹوئن) │  │   (روبوٹ       │  │   (VLA)     │ │
│  │                 │  │                 │  │   نروس سسٹم)    │  │             │ │
│  │ • ROS 2         │  │ • Gazebo        │  │ • rclpy         │  │ • آواز     │ │
│  │   فریم ورک      │  │ • Unity         │  │ • ایکشن/سروسز  │  │   پروسیسنگ  │ │
│  │ • مواصلت        │  │   انضمام        │  │ • نیوی گیشن    │  │ • کوگنیٹو   │ │
│  │ • ایکشن/سروسز  │  │ • سیمولیشن     │  │ • مینیپولیشن   │  │   منصوبہ بندی│ │
│  └─────────────────┘  └─────────────────┘  │ • سیفٹی         │  │ • وژن       │ │
│                                            │ • مواصلت       │  │   پروسیسنگ  │ │
│                                            └─────────────────┘  └─────────────┘ │
│                                                         │              │         │
│                                                         └──────────────┼─────────┘
│                                                                        ▼
│                                              ┌─────────────────────────────────┐
│                                              │        ہیومنوڈ روبوٹ           │
│                                              │                                 │
│                                              │  ┌─────────────────────────┐    │
│                                              │  │     فزیکل روبوٹ         │    │
│                                              │  │    (یا سیمولیشن)        │    │
│                                              │  │                         │    │
│                                              │  │ • لوکوموشن سسٹم        │    │
│                                              │  │ • مینیپولیشن ارمس        │    │
│                                              │  │ • سینسر سوٹ             │    │
│                                              │  │ • کمپیوٹنگ پلیٹ فارم    │    │
│                                              │  └─────────────────────────┘    │
│                                              └─────────────────────────────────┘
└─────────────────────────────────────────────────────────────────────────────────┘
```

### VLA پائپ لائن فلو

```
آواز ان پٹ ──► [Whisper] ──► [NLP] ──► [منشاء] ──► [کوگنیٹو] ──► [ٹاسک] ──► [ROS 2]
                سپیچ      قدرتی   انخلا       منصوبہ بندی    ڈیکومپوزیشن  انجام دہی
              ریکوگنیشن  زبان                              گراف          ایکشنز
                    │           │           │           │            │            │
                    ▼           ▼           ▼           ▼            ▼            ▼
              [آڈیو]    [متن]    [سیمینٹک]  [منصوبہ بندی]   [انجام دہی]   [روبوٹ]
              پروسیسنگ  تجزیہ    تجزیہ       گراف         کمانڈز        ایکشنز
                    │           │           │           │            │            │
                    └───────────┼───────────┼───────────┼────────────┼────────────┘
                                │           │           │            │
                                ▼           ▼           ▼            ▼
                            [سیاق و سباق]  [ماحول]  [اسٹیٹ]   [فیڈ بیک]
                            آگاہی         ادراک     مینجر     انضمام
                                │           │           │            │
                                └───────────┼───────────┼────────────┘
                                            │           │
                                            ▼           ▼
                                    [وژن سسٹم]  [کوآرڈینیشن]
                                    آبجیکٹ ڈیکشن  سسٹم اسٹیٹ
                                    پوز اسٹیمیشن   خامی بازیافت
                                    منظر کی سمجھ
```

## حوالہ جات

1. Brohan, C., وغیرہ (2022). "RT-1: ریل ورلڈ کنٹرول کے لیے روبوٹکس ٹرانسفارمر۔" *arXiv پری پرنٹ arXiv:2212.06817*۔

2. Ahn, M., وغیرہ (2022). "جیسا میں کر سکتا ہوں، جیسا میں کہتا ہوں نہیں: روبوٹکس افوارڈنسز میں زبان کو زمین میں اتارنا۔" *arXiv پری پرنٹ arXiv:2204.01691*۔

3. Huang, W., وغیرہ (2022). "زبان کے ماڈلز صفر-شانٹ ٹریجکٹری آپٹیمائزرز کے طور پر۔" *arXiv پری پرنٹ arXiv:2204.03535*۔

4. Chen, X., وغیرہ (2023). "زبان کے ماڈلز صفر-شانٹ منصوبہ بندی کے طور پر: امبدڈ ایجنٹس کے لیے عملی علم نکالنا۔" *بین الاقوامی کانفرنس آن مشین لرننگ (ICML) کے مکالمات*۔

5. OpenAI. (2023). "GPT-4 تکنیکی رپورٹ۔" OpenAI. دستیاب: https://openai.com/research/gpt-4

6. ROS 2 دستاویزات. (2023). "روبوٹ آپریٹنگ سسٹم 2۔" دستیاب: https://docs.ros.org/en/humble/

7. Zhu, Y., وغیرہ (2017). "گہری م reinforcement سیکھنے کا استعمال کرتے ہوئے انڈور مناظر میں ہدف-ڈرائیون وژوئل نیوی گیشن۔" *IEEE بین الاقوامی کانفرنس آن روبوٹکس اینڈ آٹومیشن (ICRA)*۔

8. Fox, D., وغیرہ (1998). "بصری ٹریکنگ کے لیے ایکٹو مارکر لوکلائزیشن۔" *IEEE بین الاقوامی کانفرنس آن روبوٹکس اینڈ آٹومیشن (ICRA)*۔

9. Kaelbling, L.P., وغیرہ (1998). "جزوی طور پر قابل مشاہدہ اسٹوکاسٹک ڈومینز میں منصوبہ بندی اور کارروائی۔" *مصنوعی ذہانت جریدہ*، 101(1-2)، 99-134۔

10. Siciliano, B., & Khatib, O. (2016). "اسپرینجر ہینڈ بک آف روبوٹکس" (2ویں ایڈیشن)۔ اسپرینجر۔