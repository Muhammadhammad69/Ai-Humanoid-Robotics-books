---
sidebar_position: 3
---

# باب 3: ہیومنوڈ روبوٹس کے لیے LLM-ڈرائیون کوگنیٹو منصوبہ بندی

## جائزہ

یہ باب ہیومنوڈ روبوٹس کے لیے بڑے زبان کے ماڈل (LLM)-ڈرائیون کوگنیٹو منصوبہ بندی سسٹم کے نفاذ کو تلاش کرتا ہے۔ کوگنیٹو منصوبہ بندی اعلی درجے کے انسانی حکم اور کم درجے کے روبوٹ ایکشن کے درمیان فاصلہ پر قابو پاتا ہے جو پیچیدہ کاموں کو قابل انجام ترتیبات میں ڈیکومپوز کرتا ہے۔ منصوبہ بندی کے لیے LLMs کا استعمال روبوٹس کو قدرتی زبان کے حکم کو سمجھنے، ماحولیاتی رکاوٹوں کے بارے میں سوچنے، اور ایسی ایکشن ترتیبات تیار کرنے کے قابل بناتا ہے جو جسمانی دنیا کی پیچیدگیوں کو مدنظر رکھتی ہیں۔ یہ باب ROS 2 کے ساتھ LLMs کو ضم کرنے کو کور کرتا ہے تاکہ ذہی، مطیع روبوٹک رویے تخلیق کیے جا سکیں۔

## سیکھنے کے اہداف

اس باب کے اختتام تک آپ درج ذیل کر سکیں گے:
- ہیومنوڈ روبوٹکس میں کوگنیٹو منصوبہ بندی کا کردار سمجھیں
- روبوٹک ایپلی کیشنز کے لیے LLM-مبنی ٹاسک ڈیکومپوزیشن نافذ کریں
- پیچیدہ کاموں کے لیے منصوبہ بندی گراف اور ایگزیکیوشن ٹریز تخلیق کریں
- ROS 2 سروسز، ایکشنز، اور ٹاپکس میں قدرتی زبان کے حکم میپ کریں
- رکاوٹوں کو ہینڈل کریں اور ماحولیاتی حالات کی بنیاد پر منصوبے ایڈجسٹ کریں
- LLM-تیار کردہ منصوبوں کے لیے سیفٹی میکانزم اور توثیق نافذ کریں
- منصوبہ ایگزیکیوشن مانیٹرنگ اور ایڈاپٹیشن کے لیے فیڈ بیک لوپس ڈیزائن کریں

## کلیدی تصورات

### روبوٹکس میں کوگنیٹو منصوبہ بندی
کوگنیٹو منصوبہ بندی اس علی درجے کی منطق کو شامل کرتا ہے جو اہداف کو قابل انجام ایکشنز میں تبدیل کرنے کے لیے درکار ہے:
- ٹاسک ڈیکومپوزیشن: پیچیدہ اہداف کو سادہ ذیلی ٹاسکس میں توڑنا
- رکاوٹ ہینڈلنگ: ماحولیاتی اور روبوٹ-مخصوص حدود کا نظم کرنا
- منصوبہ کی کارآمدی: ایکشنز کی کارآمد ترتیبات تلاش کرنا
- ایڈاپٹیشن: ایگزیکیوشن فیڈ بیک کی بنیاد پر منصوبوں کو تبدیل کرنا

### منصوبہ بندی کے لیے LLM انضمام
بڑے زبان کے ماڈل روبوٹک منصوبہ بندی میں کئی فوائد لاتے ہیں:
- قدرتی زبان کی سمجھ: براہ راست انسانی حکم کی تشریح
- عام سینس کی منطق: جنرل دنیا کے علم کا اطلاق
- اینالوجیکل منطق: نئی صورتحال میں جانے والے حل کو ایڈجسٹ کرنا
- متن کی آگاہی: صورتحال کی رکاوٹوں کو سمجھنا

### منصوبہ بندی گراف اور ایگزیکیوشن ٹریز
منصوبوں کی ساخت دار نمائندگیاں جو اس کے قابل بناتی ہیں:
- سلسلہ وار ٹاسک تنظیم
- ذیلی ٹاسکس کے درمیان انحصار ٹریکنگ
- ناکامی کی بازیافت کے لیے متبادل منصوبہ تیار کرنا
- ایگزیکیوشن مانیٹرنگ اور پیشرفت ٹریکنگ

### ROS 2 ایکشن میپنگ
LLM-تیار کردہ منصوبوں کو ROS 2 پرائمرز میں تبدیل کرنا:
- ہم وقت کارروائیوں کے لیے سروسز
- فیڈ بیک کے ساتھ طویل مدتی کاموں کے لیے ایکشنز
- مسلسل ڈیٹا سٹریمز کے لیے ٹاپکس
- کنفیگریشن کے لیے پیرامیٹر سرورز

## تکنیکی گہرائی

### کوگنیٹو منصوبہ بندی آرکیٹیکچر

LLM-ڈرائیون کوگنیٹو منصوبہ بندی سسٹم اس آرکیٹیکچر کو فالو کرتا ہے:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   قدرتی         │    │   LLM-مبنی      │    │   منصوبہ بندی   │
│   زبان کا حکم    │───▶│   ٹاسک         │───▶│   گراف          │
│                 │    │   ڈیکومپوزیشن  │    │   تیار کرنا     │
│                 │    │                 │    │                 │
│ • "کمرہ صاف     │    │ • سیمینٹک     │    │ • ایکشن         │
│   کریں"         │    │   پارسنگ       │    │   سیکوئنسنگ    │
│ • متن کی معلومات│    │ • ٹاسک         │    │ • انحصار        │
│                 │    │   شناخت        │    │   ٹریکنگ       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │                         │
                              ▼                         ▼
                    ┌─────────────────┐    ┌─────────────────┐
                    │   ماحولیاتی     │    │   ROS 2          │
                    │   متن           │    │   انضمام         │
                    │                 │    │                 │
                    │ • آبجیکٹ       │    │ • ایکشن         │
                    │   لوکیشنز      │    │   میپنگ         │
                    │ • رکاوٹیں       │    │ • سروس          │
                    │ • روبوٹ اسٹیٹ  │    │   کالز          │
                    └─────────────────┘    └─────────────────┘
                              │                         │
                              └─────────────────────────┼──────────────────┐
                                                        ▼                  ▼
                                               ┌─────────────────┐ ┌─────────────────┐
                                               │   ایگزیکیوشن    │ │   توثیق          │
                                               │   مانیٹرنگ      │ │   & سیفٹی       │
                                               │                 │ │                 │
                                               │ • پیشرفت        │ │ • رکاوٹ          │
                                               │   ٹریکنگ       │ │   چیکنگ         │
                                               │ • فیڈ بیک      │ │ • منصوبہ سیفٹی  │
                                               │   کلیکشن       │ │ • ایگزیکیوشن    │
                                               └─────────────────┘ │   توثیق          │
                                                                 └─────────────────┘
```

### LLM منصوبہ بندی نفاذ

مرکزی نفاذ میں کئی کلیدی اجزاء شامل ہیں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
import openai
import json
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum


class TaskType(Enum):
    """روبوٹک منصوبہ بندی کے لیے مختلف ٹاسک کی اقسام کی فہرست۔"""
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    PERCEPTION = "perception"
    COMMUNICATION = "communication"
    WAIT = "wait"


@dataclass
class Task:
    """منصوبہ بندی سسٹم میں ایک واحد ٹاسک کی نمائندگی۔"""
    id: str
    type: TaskType
    action: str
    parameters: Dict[str, Any]
    dependencies: List[str]
    priority: int = 1
    estimated_duration: float = 1.0  # سیکنڈ میں


@dataclass
class Plan:
    """متعدد ٹاسکس کے ساتھ ایک مکمل منصوبہ کی نمائندگی۔"""
    id: str
    original_command: str
    tasks: List[Task]
    context: Dict[str, Any]
    estimated_duration: float = 0.0


class LLMBasedPlanner(Node):
    """
    ROS 2 نوڈ جو ہیومنوڈ روبوٹس میں کوگنیٹو منصوبہ بندی کے لیے LLMs کا استعمال کرتا ہے۔
    """

    def __init__(self):
        super().__init__('llm_planner')

        # OpenAI کلائنٹ شروع کریں
        # عمل میں، آپ اسے اپنی API کلید کے ساتھ کنفیگر کریں گے
        # self.openai_client = openai.OpenAI(api_key="your-api-key")

        # پبلشرز
        self.plan_publisher = self.create_publisher(
            String,
            'robot_plan',
            10
        )

        self.action_publisher = self.create_publisher(
            String,
            'robot_action',
            10
        )

        # سبسکرائبرز
        self.command_subscriber = self.create_subscription(
            String,
            'high_level_command',
            self.command_callback,
            10
        )

        self.environment_subscriber = self.create_subscription(
            String,
            'environment_state',
            self.environment_callback,
            10
        )

        # انٹرنل اسٹیٹ
        self.current_environment = {}
        self.active_plans = {}

        # پیرامیٹرز
        self.declare_parameter('llm_model', 'gpt-4-turbo')
        self.declare_parameter('max_planning_retries', 3)
        self.declare_parameter('plan_validation_enabled', True)

        self.get_logger().info('LLM-مبنی منصوبہ ساز شروع کیا گیا')

    def command_callback(self, msg):
        """
        صارفین یا دیگر سسٹم سے اعلی درجے کے حکم کے لیے کال بیک۔
        """
        command_text = msg.data
        self.get_logger().info(f'حکم موصول ہوا: {command_text}')

        # حکم اور موجودہ ماحول کی بنیاد پر منصوبہ تیار کریں
        plan = self.generate_plan(command_text, self.current_environment)

        if plan:
            # منصوبہ شائع کریں
            plan_json = json.dumps({
                'plan_id': plan.id,
                'tasks': [
                    {
                        'id': task.id,
                        'type': task.type.value,
                        'action': task.action,
                        'parameters': task.parameters,
                        'dependencies': task.dependencies
                    } for task in plan.tasks
                ],
                'context': plan.context
            })

            plan_msg = String()
            plan_msg.data = plan_json
            self.plan_publisher.publish(plan_msg)

            self.get_logger().info(f'{len(plan.tasks)} ٹاسکس کے ساتھ منصوبہ شائع کیا گیا')
        else:
            self.get_logger().error('حکم کے لیے منصوبہ تیار کرنے میں ناکامی')

    def environment_callback(self, msg):
        """
        منصوبہ ساز کے ماحول کی سمجھ کو اپ ڈیٹ کریں۔
        """
        try:
            self.current_environment = json.loads(msg.data)
            self.get_logger().debug('ماحولیاتی حالت اپ ڈیٹ ہو گئی')
        except json.JSONDecodeError:
            self.get_logger().error('غلط ماحولیاتی حالت JSON')

    def generate_plan(self, command: str, environment: Dict[str, Any]) -> Optional[Plan]:
        """
        LLM-مبنی ٹاسک ڈیکومپوزیشن کا استعمال کرتے ہوئے ایک منصوبہ تیار کریں۔
        """
        try:
            # LLM کے لیے ایک تفصیلی پروموٹ تیار کریں
            prompt = self.create_planning_prompt(command, environment)

            # عمل میں، آپ یہاں LLM کو کال کریں گے
            # اس مثال کے لیے، ہم LLM ریسپانس کو سیمولیٹ کریں گے
            # response = self.openai_client.chat.completions.create(
            #     model=self.get_parameter('llm_model').value,
            #     messages=[{"role": "user", "content": prompt}],
            #     temperature=0.1,
            #     response_format={"type": "json_object"}
            # )
            #
            # # LLM ریسپانس کو پارس کریں
            # llm_response = response.choices[0].message.content
            # plan_data = json.loads(llm_response)

            # جمہوریت کے مقاصد کے لیے، ایک سیمولیٹڈ ریسپانس تیار کریں
            plan_data = self.simulate_llm_response(command, environment)

            # منصوبہ کی توثیق کریں
            if self.get_parameter('plan_validation_enabled').value:
                if not self.validate_plan(plan_data, environment):
                    self.get_logger().warn('تیار کردہ منصوبہ توثیق میں ناکام ہوا')
                    return None

            # LLM ریسپانس سے منصوبہ آبجیکٹ تیار کریں
            tasks = []
            for task_data in plan_data['tasks']:
                task = Task(
                    id=task_data['id'],
                    type=TaskType(task_data['type']),
                    action=task_data['action'],
                    parameters=task_data['parameters'],
                    dependencies=task_data.get('dependencies', [])
                )
                tasks.append(task)

            plan = Plan(
                id=plan_data['plan_id'],
                original_command=command,
                tasks=tasks,
                context=plan_data.get('context', {}),
                estimated_duration=plan_data.get('estimated_duration', 0.0)
            )

            return plan

        except Exception as e:
            self.get_logger().error(f'منصوبہ تیار کرنے میں خامی: {e}')
            return None

    def create_planning_prompt(self, command: str, environment: Dict[str, Any]) -> str:
        """
        منصوبہ تیار کرنے کے لیے LLM کے لیے ایک تفصیلی پروموٹ تیار کریں۔
        """
        return f"""
        آپ ایک ہیومنوڈ روبوٹ کے لیے کوگنیٹو منصوبہ بندی سسٹم ہیں۔ آپ کا کام اعلی درجے کے حکم کو قابل انجام روبوٹک ایکشنز میں ڈیکومپوز کرنا ہے۔

        ماحولیاتی متن:
        {json.dumps(environment, indent=2)}

        انجام دینے کے لیے حکم:
        "{command}"

        براہ کرم اس حکم کو روبوٹک ایکشنز کی ترتیب میں ڈیکومپوز کریں۔ ہر ایکشن کافی مخصوص ہونا چاہیے کہ روبوٹ اسے انجام دے سکے۔

        JSON فارمیٹ میں جواب دیں:
        {{
            "plan_id": "unique_plan_identifier",
            "tasks": [
                {{
                    "id": "task_unique_id",
                    "type": "navigation|manipulation|perception|communication|wait",
                    "action": "specific_action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "dependencies": ["task_id_1", "task_id_2"]  // ٹاسکس جو اس سے پہلے مکمل ہونے چاہئیں
                }}
            ],
            "context": {{"additional_context": "information"}},
            "estimated_duration": 120.0  // سیکنڈ میں تخمینی وقت
        }}

        ہدایات:
        1. پیچیدہ کاموں کو چھوٹے، قابل انجام اقدامات میں توڑیں
        2. متن سے ماحولیاتی رکاوٹوں پر غور کریں
        3. جب روبوٹ کو اس کے ماحول کا جائزہ لینے کی ضرورت ہو تو ادراک ٹاسکس شامل کریں
        4. مقامات کے درمیان حرکت کے لیے نیوی گیشن ٹاسکس شامل کریں
        5. آبجیکٹ تعامل کے لیے مینیپولیشن ٹاسکس شامل کریں
        6. جہاں ترتیب اہم ہو وہاں ٹاسکس کے درمیان انحصار شامل کریں
        7. ہر ٹاسک کے لیے مخصوص، ایکشن لینے والی زبان استعمال کریں
        """

    def simulate_llm_response(self, command: str, environment: Dict[str, Any]) -> Dict[str, Any]:
        """
        جمہوریت کے مقاصد کے لیے LLM ریسپانس کی تقلید کریں۔
        ایک حقیقی نفاذ میں، اس کی جگہ حقیقی LLM کال لے گی۔
        """
        # یہ ایک سادہ سیمولیشن ہے - عمل میں، LLM زیادہ جامع منصوبے تیار کرے گا
        command_lower = command.lower()

        if "clean the room" in command_lower:
            return {
                "plan_id": f"plan_{int(time.time())}",
                "tasks": [
                    {
                        "id": "task_1",
                        "type": "perception",
                        "action": "scan_room",
                        "parameters": {},
                        "dependencies": []
                    },
                    {
                        "id": "task_2",
                        "type": "navigation",
                        "action": "navigate_to_location",
                        "parameters": {"x": 1.0, "y": 2.0, "theta": 0.0},
                        "dependencies": ["task_1"]
                    },
                    {
                        "id": "task_3",
                        "type": "perception",
                        "action": "detect_trash",
                        "parameters": {},
                        "dependencies": ["task_2"]
                    },
                    {
                        "id": "task_4",
                        "type": "manipulation",
                        "action": "pickup_object",
                        "parameters": {"object_id": "trash_1"},
                        "dependencies": ["task_3"]
                    },
                    {
                        "id": "task_5",
                        "type": "navigation",
                        "action": "navigate_to_bin",
                        "parameters": {"x": 0.0, "y": 0.0, "theta": 0.0},
                        "dependencies": ["task_4"]
                    },
                    {
                        "id": "task_6",
                        "type": "manipulation",
                        "action": "dispose_object",
                        "parameters": {"object_id": "trash_1"},
                        "dependencies": ["task_5"]
                    }
                ],
                "context": {"original_command": command},
                "estimated_duration": 300.0
            }
        elif "bring me" in command_lower or "get me" in command_lower:
            return {
                "plan_id": f"plan_{int(time.time())}",
                "tasks": [
                    {
                        "id": "task_1",
                        "type": "perception",
                        "action": "locate_object",
                        "parameters": {"object_type": "requested_item"},
                        "dependencies": []
                    },
                    {
                        "id": "task_2",
                        "type": "navigation",
                        "action": "navigate_to_object",
                        "parameters": {"x": 1.5, "y": 2.5, "theta": 0.0},
                        "dependencies": ["task_1"]
                    },
                    {
                        "id": "task_3",
                        "type": "manipulation",
                        "action": "grasp_object",
                        "parameters": {"object_id": "requested_item"},
                        "dependencies": ["task_2"]
                    },
                    {
                        "id": "task_4",
                        "type": "navigation",
                        "action": "navigate_to_user",
                        "parameters": {"x": 0.0, "y": 0.0, "theta": 0.0},
                        "dependencies": ["task_3"]
                    },
                    {
                        "id": "task_5",
                        "type": "manipulation",
                        "action": "release_object",
                        "parameters": {"object_id": "requested_item"},
                        "dependencies": ["task_4"]
                    }
                ],
                "context": {"original_command": command},
                "estimated_duration": 180.0
            }
        else:
            # غیر پہچانے گئے حکم کے لیے ڈیفالٹ ریسپانس
            return {
                "plan_id": f"plan_{int(time.time())}",
                "tasks": [
                    {
                        "id": "task_1",
                        "type": "communication",
                        "action": "request_clarification",
                        "parameters": {"message": f"میں یہ انجام دینے کا طریقہ نہیں سمجھ سکتا: {command}"},
                        "dependencies": []
                    }
                ],
                "context": {"original_command": command},
                "estimated_duration": 10.0
            }

    def validate_plan(self, plan_data: Dict[str, Any], environment: Dict[str, Any]) -> bool:
        """
        سیفٹی اور قابلیت کے لیے تیار کردہ منصوبہ کی توثیق کریں۔
        """
        try:
            # چیک کریں کہ تمام ضروری فیلڈز موجود ہیں
            required_fields = ['plan_id', 'tasks']
            for field in required_fields:
                if field not in plan_data:
                    self.get_logger().error(f'ضروری فیلڈ غائب ہے: {field}')
                    return False

            # ہر ٹاسک کی توثیق کریں
            for task in plan_data['tasks']:
                required_task_fields = ['id', 'type', 'action', 'parameters']
                for field in required_task_fields:
                    if field not in task:
                        self.get_logger().error(f'ٹاسک {task.get("id", "unknown")} فیلڈ غائب ہے: {field}')
                        return False

                # ٹاسک ٹائپ کی توثیق کریں
                try:
                    TaskType(task['type'])
                except ValueError:
                    self.get_logger().error(f'غلط ٹاسک ٹائپ: {task["type"]}')
                    return False

                # اضافی سیفٹی چیکس یہاں جا سکتی ہیں
                # مثال کے طور پر، چیک کریں کہ نیوی گیشن ٹارگٹس محفوظ علاقوں میں ہیں
                # یا کہ مینیپولیشن ایکشنز روبوٹ کی صلاحیتوں کے تحت قابل عمل ہیں

            return True

        except Exception as e:
            self.get_logger().error(f'منصوبہ کی توثیق میں خامی: {e}')
            return False


def main(args=None):
    """
    LLM-مبنی منصوبہ ساز نوڈ چلانے کے لیے مرکزی فنکشن۔
    """
    rclpy.init(args=args)

    planner = LLMBasedPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('صارف کی طرف سے مداخلت، بند کیا جا رہا ہے...')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## کوڈ کی مثالیں

### رکاوٹ ہینڈلنگ کے ساتھ اعلی درجے کی منصوبہ بندی

```python
from typing import List, Dict, Any, Optional, Tuple
import networkx as nx  # منصوبہ بندی گراف کی نمائندگی کے لیے


class AdvancedLLMPlanner(LLMBasedPlanner):
    """
    اعلی درجے کی رکاوٹ ہینڈلنگ اور کارآمدی کے ساتھ بہتر LLM منصوبہ ساز۔
    """

    def __init__(self):
        super().__init__()

        # انحصار کے انتظام کے لیے منصوبہ بندی گراف
        self.planning_graph = nx.DiGraph()

    def generate_adaptive_plan(self, command: str, environment: Dict[str, Any]) -> Optional[Plan]:
        """
        ماحولیاتی رکاوٹوں اور روبوٹ کی صلاحیتوں کے مطابق ایڈجسٹ ہونے والا ایک منصوبہ تیار کریں۔
        """
        # ماحولیاتی رکاوٹوں کا تجزیہ کریں
        constraints = self.analyze_constraints(environment)

        # ابتدائی منصوبہ تیار کریں
        initial_plan = self.generate_plan(command, environment)

        if not initial_plan:
            return None

        # رکاوٹوں کی بنیاد پر منصوبہ ایڈجسٹ کریں
        adapted_plan = self.adapt_plan_to_constraints(initial_plan, constraints, environment)

        return adapted_plan

    def analyze_constraints(self, environment: Dict[str, Any]) -> Dict[str, Any]:
        """
        منصوبہ بندی کو متاثر کرنے والی ماحولیاتی اور روبوٹ کی رکاوٹوں کا تجزیہ کریں۔
        """
        constraints = {
            'navigation': {
                'blocked_areas': environment.get('blocked_areas', []),
                'narrow_passages': environment.get('narrow_passages', []),
                'climbing_limitations': environment.get('climbing_limitations', 0.1)  # میٹر
            },
            'manipulation': {
                'reachable_objects': environment.get('reachable_objects', []),
                'graspable_objects': environment.get('graspable_objects', []),
                'weight_limit': environment.get('weight_limit', 5.0)  # kg
            },
            'perception': {
                'visible_objects': environment.get('visible_objects', []),
                'occluded_areas': environment.get('occluded_areas', [])
            },
            'safety': {
                'forbidden_zones': environment.get('forbidden_zones', []),
                'safe_distance': environment.get('safe_distance', 0.5)  # میٹر
            }
        }

        return constraints

    def adapt_plan_to_constraints(self, plan: Plan, constraints: Dict[str, Any],
                                 environment: Dict[str, Any]) -> Plan:
        """
        پہچانی گئی رکاوٹوں کے ساتھ منصوبہ ایڈجسٹ کریں۔
        """
        adapted_tasks = []

        for task in plan.tasks:
            # چیک کریں کہ کیا ٹاسک رکاوٹوں کو مدنظر رکھتے ہوئے قابل عمل ہے
            if self.is_task_feasible(task, constraints, environment):
                adapted_tasks.append(task)
            else:
                # متبادل ٹاسک تیار کریں یا موجودہ ٹاسک میں تبدیلی کریں
                alternative_tasks = self.generate_task_alternatives(task, constraints, environment)
                adapted_tasks.extend(alternative_tasks)

        # تبدیلیوں کی بنیاد پر انحصار اپ ڈیٹ کریں
        updated_tasks = self.update_task_dependencies(adapted_tasks)

        return Plan(
            id=f"adapted_{plan.id}",
            original_command=plan.original_command,
            tasks=updated_tasks,
            context={**plan.context, "adapted_due_to_constraints": True},
            estimated_duration=self.estimate_plan_duration(updated_tasks)
        )

    def is_task_feasible(self, task: Task, constraints: Dict[str, Any],
                        environment: Dict[str, Any]) -> bool:
        """
        موجودہ رکاوٹوں کو مدنظر رکھتے ہوئے چیک کریں کہ کیا ٹاسک قابل عمل ہے۔
        """
        if task.type == TaskType.NAVIGATION:
            target_location = task.parameters.get('x'), task.parameters.get('y')
            if target_location in constraints['navigation']['blocked_areas']:
                return False

        elif task.type == TaskType.MANIPULATION:
            object_id = task.parameters.get('object_id')
            if object_id and object_id not in constraints['manipulation']['graspable_objects']:
                return False

        elif task.type == TaskType.PERCEPTION:
            target_area = task.parameters.get('target_area')
            if target_area in constraints['perception']['occluded_areas']:
                return False

        return True

    def generate_task_alternatives(self, task: Task, constraints: Dict[str, Any],
                                  environment: Dict[str, Any]) -> List[Task]:
        """
        جب اصل ٹاسک قابل عمل نہ ہو تو متبادل ٹاسکس تیار کریں۔
        """
        alternatives = []

        if task.type == TaskType.NAVIGATION:
            # متبادل راستہ تلاش کریں
            alternative_route = self.find_alternative_navigation(task, constraints)
            if alternative_route:
                alternatives.extend(alternative_route)
            else:
                # "درخواست مدد" ٹاسک شامل کریں
                alternatives.append(Task(
                    id=f"alt_{task.id}",
                    type=TaskType.COMMUNICATION,
                    action="request_assistance",
                    parameters={"message": f"میں {task.parameters.get('x', 'unknown')} پر جانے میں ناکام"},
                    dependencies=task.dependencies
                ))

        elif task.type == TaskType.MANIPULATION:
            # متبادل آبجیکٹ یا نقطہ نظر تلاش کریں
            alternatives.append(Task(
                id=f"alt_{task.id}",
                type=TaskType.COMMUNICATION,
                action="request_alternative",
                parameters={"message": f"آبجیکٹ {task.parameters.get('object_id')} کو مینیپولیٹ نہیں کر سکتا"},
                dependencies=task.dependencies
            ))

        return alternatives

    def find_alternative_navigation(self, task: Task, constraints: Dict[str, Any]) -> Optional[List[Task]]:
        """
        جب براہ راست راستہ بند ہو تو متبادل نیوی گیشن راستہ تلاش کریں۔
        """
        # یہ A* یا RRT جیسے راستہ منصوبہ بندی الگوری دھم نافذ کرے گا
        # اس مثال کے لیے، ہم ایک سادہ متبادل لوٹائیں گے
        target_x = task.parameters.get('x', 0)
        target_y = task.parameters.get('y', 0)

        # چیک کریں کہ کیا ایک سادہ متبادل راستہ ہے
        # عمل میں، یہ مناسب راستہ منصوبہ بندی استعمال کرے گا
        alternative_tasks = [
            Task(
                id=f"alt_nav_1_{task.id}",
                type=TaskType.NAVIGATION,
                action="navigate_via_intermediate",
                parameters={"x": target_x - 1.0, "y": target_y, "theta": 0.0},
                dependencies=task.dependencies
            ),
            Task(
                id=f"alt_nav_2_{task.id}",
                type=TaskType.NAVIGATION,
                action="navigate_to_target",
                parameters={"x": target_x, "y": target_y, "theta": 0.0},
                dependencies=[f"alt_nav_1_{task.id}"]
            )
        ]

        return alternative_tasks

    def update_task_dependencies(self, tasks: List[Task]) -> List[Task]:
        """
        تبدیلیوں کے بعد ٹاسک انحصار اپ ڈیٹ کریں۔
        """
        # یہ نئے ٹاسک سیکوئنس کی بنیاد پر انحصار اپ ڈیٹ کرے گا
        # فی الحال، ہم ممکنہ حد تک اصل انحصار سٹرکچر کو برقرار رکھیں گے
        return tasks

    def estimate_plan_duration(self, tasks: List[Task]) -> float:
        """
        منصوبہ کا کل دورانیہ تخمینہ لگائیں۔
        """
        return sum(task.estimated_duration for task in tasks)


# اعلی درجے کے منصوبہ ساز کی مثال کا استعمال
def create_advanced_planner_node():
    """
    اعلی درجے کا منصوبہ ساز نوڈ انسٹنس تخلیق کریں اور لوٹائیں۔
    """
    return AdvancedLLMPlanner()
```

## ڈائریم (متن-مبنی)

### منصوبہ بندی عمل کا فلو

```
اعلی درجے کا حکم ──► [LLM تشریح] ──► [ٹاسک ڈیکومپوزیشن] ──► [رکاوٹ چیکنگ]
       │                       │                        │                       │
       │                       ▼                        ▼                       ▼
       │               [سیمینٹک پارسنگ]      [ایکشن سیکوئنسنگ]    [قابلیت تجزیہ]
       │               [منشاء انخلا]         [انحصار میپنگ]      [سیفٹی توثیق]
       │                                                                 │
       └─────────────────────────────────────────────────────────────────┼─────────► منصوبہ
                                                                         ▼
                                                                 [منصوبہ کارآمدی]
                                                                 [متبادل تیار کرنا]
```

### منصوبہ بندی گراف سٹرکچر

```
                    ROOT (اصل حکم)
                           │
              ┌────────────┼────────────┐
              │            │            │
        نیوی گیشن      ادراک        مینیپولیشن
        ٹاسک A        ٹاسک B        ٹاسک C
              │            │            │
        ┌─────┘            │            └─────┐
        │                  │                  │
    دروازہ کی حالت     علاقہ میں آبجیکٹ    گریپر کے ساتھ
    چیک کریں          ڈیٹیکٹ کریں        آبجیکٹ گریپ کریں
        │                  │                  │
        └──────────────────┼──────────────────┘
                           ▼
                    دنیا کا ماڈل اپ ڈیٹ کریں
                           │
                           ▼
                    ایکشن منصوبہ انجام دیں
```

## عام مسائل

### 1. بہت پیچیدہ منصوبے
**مسئلہ**: LLMs بہت پیچیدہ منصوبے تیار کر سکتے ہیں جنہیں انجام دینا مشکل ہے۔
**حل**: قابلیت کو یقینی بنانے کے لیے منصوبہ سادگی اور توثیق کے اقدامات نافذ کریں۔

### 2. ماحولیاتی متن کی کمی
**مسئلہ**: موجودہ ماحولیاتی حالت کو مدنظر نہ رکھتے ہوئے منصوبے تیار کیے جاتے ہیں۔
**حل**: یقینی بنائیں کہ LLM کو تازہ ترین ماحولیاتی معلومات تک رسائی حاصل ہو۔

### 3. سیفٹی کی خلاف ورزیاں
**مسئلہ**: LLM-تیار کردہ منصوبے غیر محفوظ ایکشنز شامل کر سکتے ہیں۔
**حل**: سیفٹی توثیق لیئرز نافذ کریں جو ایگزیکیوشن سے پہلے تمام ایکشنز کو چیک کریں۔

### 4. ایگزیکیوشن ناکامی کی بازیافت
**مسئلہ**: منصوبے ممکنہ ایگزیکیوشن ناکامیوں کا احاطہ نہیں کرتے۔
**حل**: منصوبہ بندی کے عمل میں مانیٹرنگ اور بازیافت کے میکانزم شامل کریں۔

### 5. کمپیوٹیشنل اضافی بوجھ
**مسئلہ**: پیچیدہ LLM کویریز قابل قبول لیٹنسی پیدا کر سکتی ہیں۔
**حل**: عام منصوبوں کو کیش کریں اور سادہ ٹاسکس کے لیے ہلکی توثیق استعمال کریں۔

## چیک پوائنٹس

### سمجھ کا چیک 1: منصوبہ بندی کے تصورات
- ٹاسک ڈیکومپوزیشن اور ایکشن انجام دہی میں کیا فرق ہے؟
- روایتی طریقوں کے مقابلے میں LLMs روبوٹک منصوبہ بندی کو کیسے بہتر بناتے ہیں؟
- منصوبہ بندی گراف کے کلیدی اجزاء کیا ہیں؟

### سمجھ کا چیک 2: نفاذ
- آپ منصوبہ ساز کو متعدد روبوٹ کوآرڈینیشن کے لیے کیسے تبدیل کریں گے؟
- آپ نقصان دہ روبوٹ ایکشنز کو روکنے کے لیے کون سی سیفٹی چیکس شامل کریں گے؟
- آپ حقیقی وقت کی ایپلی کیشنز کے لیے منصوبہ بندی کے عمل کو کیسے بہتر بنائیں گے؟

### اطلاق کا چیک: کوگنیٹو منصوبہ بندی
- آپ منصوبہ ساز کو تعاونی کاموں کے لیے کیسے بڑھائیں گے؟
- کون سے اضافی ماحولیاتی سینسر منصوبہ بندی کی درستگی کو بہتر بنائیں گے؟
- آپ متعدد صارفین کے متضاد حکم کو کیسے ہینڈل کریں گے؟

## حوالہ جات

1. Chen, X., وغیرہ (2023). "زیرو-شانٹ پلینرز کے طور پر زبان کے ماڈلز: جسمانی ایجنٹس کے لیے قابل عمل علم نکالنا۔" *بین الاقوامی کانفرنس آن مشین لرننگ (ICML) کے مکالمات*۔

2. Huang, W., وغیرہ (2022). "زبان کے ماڈلز کے طور پر زیرو-شانٹ ٹریجکٹری آپٹیمائزرز۔" *arXiv پری پرنٹ arXiv:2204.03535*۔

3. OpenAI. (2023). "GPT-4 تکنیکی رپورٹ۔" OpenAI. دستیاب: https://openai.com/research/gpt-4

4. Brohan, C., وغیرہ (2022). "ریل-ورلڈ کنٹرول کے لیے روبوٹکس ٹرانسفارمر۔" *arXiv پری پرنٹ arXiv:2212.06817*۔

5. Ahn, M., وغیرہ (2022). "میں جو کہوں اس کے بجائے میں جو کروں: روبوٹک ایفورڈنسز میں زبان کو زمین میں اتارنا۔" *arXiv پری پرنٹ arXiv:2204.01691*۔

6. Zhu, Y., وغیرہ (2017). "گہری مضبوطی سیکھنے کا استعمال کرتے ہوئے انڈور مناظر میں ہدف-ڈرائیون وژوئل نیوی گیشن۔" *IEEE بین الاقوامی کانفرنس آن روبوٹکس اینڈ آٹومیشن (ICRA)*۔

7. Fox, D., وغیرہ (1998). "ویژوئل ٹریکنگ کے لیے ایکٹو مارکر لوکلائزیشن۔" *IEEE بین الاقوامی کانفرنس آن روبوٹکس اینڈ آٹومیشن (ICRA)*۔

8. Kaelbling, L.P., وغیرہ (1998). "جزوی طور پر قابل مشاہدہ اسٹوکاسٹک ڈومینز میں منصوبہ بندی اور کارروائی۔" *مصنوعی ذہانت جریدہ*، 101(1-2)، 99-134۔

9. Russell, S., & Norvig, P. (2020). "مصنوعی ذہانت: ایک جدید نقطہ نظر" (4ویں ایڈیشن)۔ پئرسن۔

10. Siciliano, B., & Khatib, O. (2016). "سپرنجر ہینڈ بک آف روبوٹکس" (2ویں ایڈیشن)۔ سپرنجر۔