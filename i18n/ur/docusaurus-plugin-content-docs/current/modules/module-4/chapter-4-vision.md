---
sidebar_position: 4
---

# باب 4: ہیومنوڈ روبوٹ ادراک کے لیے وژن انضمام

## جائزہ

یہ باب ہیومنوڈ روبوٹس کے لیے کمپیوٹر وژن سسٹم کو زبان کی سمجھ کے ساتھ ضم کرنے کو کور کرتا ہے۔ وژن انضمام روبوٹس کو اشیاء کو سمجھنے، پوز اسٹیمیٹ کرنے، اور منظروں کو اس کے تناظر میں سمجھنے کے قابل بناتا ہے کہ الفاظ کے حکم ہیں۔ یہ باب اشیاء کا پتہ لگانا، پوز اسٹیمیشن، منظر کی سمجھ، اور وژن-زبان گراؤنڈنگ تکنیکوں کو تلاش کرتا ہے جو روبوٹس کو بصری ادراک کو قدرتی زبان کے حکم سے جوڑنے کے قابل بناتی ہیں۔ یہ صلاحیت ایسے روبوٹس کو تخلیق کرنے کے لیے ضروری ہے جو انسانی ماحول میں مؤثر طریقے سے کام کر سکیں اور جگہی حوالے والے حکم کے جواب میں کام کر سکیں۔

## سیکھنے کے اہداف

اس باب کے اختتام تک آپ درج ذیل کر سکیں گے:
- روبوٹک ایپلی کیشنز کے لیے اشیاء کا پتہ لگانے والا سسٹم نافذ کریں
- مینیپولیشن ٹاسکس کے لیے پوز اسٹیمیٹ کریں
- جگہی منطق کے لیے منظر کی سمجھ کی صلاحیتیں تخلیق کریں
- حکم کی تشریح کے لیے وژن-زبان گراؤنڈنگ ضم کریں
- متعدد ادراک موڈلز کو جوڑنے کے لیے سینسر فیوژن تکنیکوں کا اطلاق کریں
- روبوٹک وژن میں عام چیلنجوں کو ہینڈل کریں جیسے کہ لائٹنگ کی تبدیلیاں اور اوکلیوژنز
- حقیقی وقت کے روبوٹک آپریشن کی حمایت کرنے والے وژن سسٹم ڈیزائن کریں
- وژن-مبنی روبوٹ کنٹرول کے لیے سیفٹی میکانزم نافذ کریں

## کلیدی تصورات

### روبوٹکس میں اشیاء کا پتہ لگانا
روبوٹک ادراک کے لیے اشیاء کا پتہ لگانا بنیادی ہے، جو روبوٹس کو اپنے ماحول میں اشیاء کی شناخت اور مقام کو تلاش کرنے کے قابل بناتا ہے:
- تعاملی روبوٹکس کے لیے حقیقی وقت کی پروسیسنگ کی ضروریات
- لائٹنگ اور ماحولیاتی تبدیلیوں کے خلاف مزاحمت
- ایکشن منصوبہ بندی کے لیے روبوٹ کوآرڈینیٹ سسٹم کے ساتھ انضمام
- متحرک ماحول کے لیے مسلسل ٹریکنگ

### پوز اسٹیمیشن
مینیپولیشن ٹاسکس کے لیے درست پوز اسٹیمیشن انتہائی ضروری ہے:
- درست مینیپولیشن کے لیے 6-DOF پوز اسٹیمیشن
- کیمرہ اور روبوٹ فریموں کے درمیان کوآرڈینیٹ سسٹم ٹرانسفارمیشنز
- محفوظ مینیپولیشن کے لیے غیر یقینی کی مقدار
- بہتر درستگی کے لیے متعدد نظروں کا فیوژن

### منظر کی سمجھ
منظر کی سمجھ اشیاء کا پتہ لگانے سے آگے بڑھ کر متن کی آگاہی فراہم کرتی ہے:
- اشیاء کے درمیان جگہی تعلقات
- اشیاء کی ترتیبات کی کاروائی کی سمجھ
- سرگرمی کی شناخت اور منظر کا متن
- نیوی گیشن کی جگہ کی شناخت

### وژن-زبان گراؤنڈنگ
بصری ادراک کو زبان کی سمجھ سے جوڑنا:
- بصری مناظر میں اشیاء کے حوالہ جات کو زمین میں اتارنا
- جگہی زبان کی تشریح (بائیں، دائیں، قریب، دور)
- بصری متن کے ذریعے مبہم زبان کو ہینڈل کرنا
- ملٹی-موڈل اٹینشن میکانزمز

## تکنیکی گہرائی

### وژن سسٹم آرکیٹیکچر

انضمام شدہ وژن سسٹم اس آرکیٹیکچر کو فالو کرتا ہے:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   کیمرہ         │───▶│   اشیاء کا      │───▶│   پوز           │
│   ان پٹ         │    │   پتہ لگانا     │    │   اسٹیمیشن     │
│                 │    │                 │    │                 │
│ • RGB تصاویر   │    │ • YOLO/         │    │ • 6-DOF         │
│ • ڈیپتھ ڈیٹا   │    │   Detectron2    │    │   اسٹیمیشن     │
│ • متعدد        │    │ • کمپلیمنس     │    │ • کوآرڈینیٹ   │
│   نظروں         │    │   اسکورنگ      │    │   ٹرانسفارم   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │                         │
                              ▼                         ▼
                    ┌─────────────────┐    ┌─────────────────┐
                    │   منظر          │    │   گراؤنڈنگ      │
                    │   سمجھ           │    │   پروسیسنگ      │
                    │                 │    │                 │
                    │ • جگہی         │    │ • زبان-         │
                    │   تعلقات       │    │   وژن           │
                    │ • کاروائی       │    │   الائمنٹ      │
                    │   متن           │    │ • اٹینشن       │
                    │ • سرگرمی       │    │   میکانزمز      │
                    │   کی شناخت      │    └─────────────────┘
                    └─────────────────┘              │
                              │                      ▼
                              └──────────────────────┼──────────────────┐
                                                     ▼                  ▼
                                            ┌─────────────────┐ ┌─────────────────┐
                                            │   ROS 2         │ │   ایکشن         │
                                            │   انضمام        │ │   منصوبہ بندی   │
                                            │                 │ │                 │
                                            │ • ٹاپک          │ │ • ٹاسک          │
                                            │   شائع کرنا     │ │   ڈیکومپوزیشن  │
                                            │ • سروس          │ │ • رکاوٹ          │
                                            │   کالز          │ │   ہینڈلنگ      │
                                            └─────────────────┘ └─────────────────┘
```

### وژن پروسیسنگ نفاذ

مرکزی وژن پروسیسنگ سسٹم متعدد منسلک اجزاء پر مشتمل ہے:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String
import cv2
import numpy as np
import torch
import torchvision.transforms as T
from cv_bridge import CvBridge
import json
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
import threading
import queue


@dataclass
class DetectedObject:
    """منظر میں ایک پتہ چلی ہوئی شے کی نمائندگی۔"""
    id: str
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x, y, width, height)
    center_2d: Tuple[int, int]       # تصویر کے کوآرڈینیٹس میں 2D مرکز
    center_3d: Optional[Tuple[float, float, float]] = None  # دنیا کے کوآرڈینیٹس میں 3D مرکز
    pose_3d: Optional[Dict[str, float]] = None  # 6-DOF پوز اگر دستیاب ہو


class VisionProcessor(Node):
    """
    ہیومنوڈ روبوٹس میں کمپیوٹر وژن پروسیسنگ کے لیے ROS 2 نوڈ۔
    اشیاء کا پتہ لگانا، پوز اسٹیمیشن، اور منظر کی سمجھ کو ہینڈل کرتا ہے۔
    """

    def __init__(self):
        super().__init__('vision_processor')

        # وژن کمپوننٹس شروع کریں
        self.bridge = CvBridge()

        # اشیاء کا پتہ لگانے والا ماڈل لوڈ کریں (TorchVision کے پری-ٹرینڈ ماڈل کو مثال کے طور پر استعمال کریں)
        self.detection_model = self.load_detection_model()
        self.detection_transform = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # پوز اسٹیمیشن ماڈل شروع کریں (سادہ مثال)
        self.pose_model = self.load_pose_model()

        # تصویر پروسیسنگ کی قطار
        self.image_queue = queue.Queue(maxsize=10)

        # پبلشرز
        self.detections_publisher = self.create_publisher(
            String,
            'vision/detections',
            10
        )

        self.poses_publisher = self.create_publisher(
            String,
            'vision/poses',
            10
        )

        self.scene_publisher = self.create_publisher(
            String,
            'vision/scene',
            10
        )

        # سبسکرائبرز
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.vision_command_subscriber = self.create_subscription(
            String,
            'vision/command',
            self.vision_command_callback,
            10
        )

        # انٹرنل اسٹیٹ
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.latest_image = None
        self.processing_thread = None
        self.shutdown_requested = False

        # پیرامیٹرز
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('enable_pose_estimation', True)
        self.declare_parameter('enable_scene_understanding', True)
        self.declare_parameter('max_detection_objects', 20)

        # پروسیسنگ تھریڈ شروع کریں
        self.processing_thread = threading.Thread(target=self.process_images, daemon=True)
        self.processing_thread.start()

        self.get_logger().info('وژن پروسیسر شروع کیا گیا')

    def load_detection_model(self):
        """
        اشیاء کا پتہ لگانے والا ماڈل لوڈ کریں۔
        عمل میں، یہ YOLO، Detectron2، یا دیگر ماڈلز ہو سکتے ہیں۔
        """
        # اس مثال کے لیے، ہم ایک torchvision ماڈل استعمال کریں گے
        # عمل میں، آپ YOLOv5/v8، Detectron2، وغیرہ لوڈ کر سکتے ہیں
        try:
            model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            model.eval()
            return model
        except Exception as e:
            self.get_logger().warn(f'YOLOv5 ماڈل لوڈ کرنے میں ناکامی: {e}')
            # فال بیک ایک سادہ پلیس ہولڈر کے لیے
            return None

    def load_pose_model(self):
        """
        پوز اسٹیمیشن ماڈل لوڈ کریں۔
        """
        # پوز اسٹیمیشن ماڈل کے لیے پلیس ہولڈر
        return None

    def image_callback(self, msg):
        """
        آنے والی کیمرہ تصاویر کے لیے کال بیک۔
        """
        try:
            # ROS تصویر کو OpenCV فارمیٹ میں تبدیل کریں
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image

            # پروسیسنگ قطار میں شامل کریں
            if not self.image_queue.full():
                self.image_queue.put((msg.header.stamp, cv_image))
            else:
                # قطار بھری ہوئی ہے تو سب سے پرانی تصویر ڈراپ کریں
                try:
                    self.image_queue.get_nowait()
                    self.image_queue.put((msg.header.stamp, cv_image))
                except queue.Empty:
                    pass

        except Exception as e:
            self.get_logger().error(f'تصویر پروسیسنگ میں خامی: {e}')

    def camera_info_callback(self, msg):
        """
        کیمرہ کیلیبریشن معلومات کے لیے کال بیک۔
        """
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def vision_command_callback(self, msg):
        """
        وژن پروسیسنگ کمانڈز کو ہینڈل کریں۔
        """
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '')

            if command == 'detect_objects':
                self.get_logger().info('اشیاء کا پتہ لگانے کا حکم موصول ہوا')
                # اگر تصویر دستیاب ہو تو فوری پروسیسنگ ٹریگر کریں
                if self.latest_image is not None:
                    self.process_single_image(self.latest_image)

            elif command == 'estimate_poses':
                self.get_logger().info('پوز اسٹیمیشن کا حکم موصول ہوا')
                # پوز اسٹیمیشن کے لیے اضافی پروسیسنگ

        except json.JSONDecodeError:
            self.get_logger().error('غلط وژن کمانڈ JSON')

    def process_images(self):
        """
        قطار سے تصاویر کو مسلسل پروسیس کریں۔
        """
        while not self.shutdown_requested:
            try:
                # قطار سے ٹائم آؤٹ کے ساتھ تصویر حاصل کریں
                timestamp, image = self.image_queue.get(timeout=0.1)

                # تصویر کو پروسیس کریں
                self.process_single_image(image)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'تصویر پروسیسنگ لوپ میں خامی: {e}')

    def process_single_image(self, image):
        """
        اشیاء کا پتہ لگانے اور پوز اسٹیمیشن کے لیے ایک تصویر کو پروسیس کریں۔
        """
        try:
            # اشیاء کا پتہ لگائیں
            detections = self.detect_objects(image)

            # اگر کیمرہ انفارمیشن دستیاب ہو، 3D پوزیشنز کمپیوٹ کریں
            if self.camera_matrix is not None and self.distortion_coeffs is not None:
                detections = self.compute_3d_positions(detections, image)

            # اگر فعال ہو تو پوز اسٹیمیشن کریں
            if self.get_parameter('enable_pose_estimation').value:
                detections = self.estimate_poses(detections, image)

            # پتہ چلنے والی اشیاء کے نتائج شائع کریں
            self.publish_detections(detections)

            # اگر فعال ہو تو منظر کی سمجھ کریں
            if self.get_parameter('enable_scene_understanding').value:
                scene_description = self.understand_scene(detections, image)
                self.publish_scene_description(scene_description)

        except Exception as e:
            self.get_logger().error(f'ایک تصویر کی پروسیسنگ میں خامی: {e}')

    def detect_objects(self, image) -> List[DetectedObject]:
        """
        تصویر پر اشیاء کا پتہ لگائیں۔
        """
        if self.detection_model is None:
            # فال بیک: کوئی پتہ چلنے والی اشیاء نہیں لوٹائیں
            return []

        try:
            # ماڈل ان پٹ کے لیے تصویر کو تبدیل کریں
            img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.detection_model(img_rgb)

            # پتہ چلنے والے نتائج کو پارس کریں
            detections = []
            for i, det in enumerate(results.xyxy[0]):  # xyxy فارمیٹ: [x1, y1, x2, y2, conf, class]
                x1, y1, x2, y2, conf, cls = det
                x1, y1, x2, y2, conf, cls = int(x1), int(y1), int(x2), int(y2), float(conf), int(cls)

                # کلاس نام حاصل کریں (سادہ - عمل میں، مناسب کلاس میپنگ استعمال کریں)
                class_names = self.detection_model.names if hasattr(self.detection_model, 'names') else {}
                class_name = class_names.get(cls, f'unknown_{cls}')

                # کمپلیمنس کی حد چیک کریں
                if conf >= self.get_parameter('detection_threshold').value:
                    width = x2 - x1
                    height = y2 - y1
                    center_2d = ((x1 + x2) // 2, (y1 + y2) // 2)

                    obj = DetectedObject(
                        id=f"obj_{len(detections)}",
                        class_name=class_name,
                        confidence=conf,
                        bbox=(x1, y1, width, height),
                        center_2d=center_2d
                    )
                    detections.append(obj)

                    # پتہ چلنے والی اشیاء کی تعداد کو محدود کریں
                    if len(detections) >= self.get_parameter('max_detection_objects').value:
                        break

            return detections

        except Exception as e:
            self.get_logger().error(f'اشیاء کے پتہ لگانے میں خامی: {e}')
            return []

    def compute_3d_positions(self, detections: List[DetectedObject], image) -> List[DetectedObject]:
        """
        کیمرہ کیلیبریشن کا استعمال کرتے ہوئے 2D پتے سے 3D پوزیشنز کمپیوٹ کریں۔
        """
        if not detections or self.camera_matrix is None:
            return detections

        updated_detections = []
        for det in detections:
            # سادہ ڈیپتھ اسٹیمیشن (عمل میں، سٹیریو وژن یا ڈیپتھ سینسر استعمال کریں)
            # فی الحال، ہم ایک پلیس ہولڈر ڈیپتھ ویلیو استعمال کریں گے
            # حقیقی نفاذ میں، آپ اصل ڈیپتھ ڈیٹا استعمال کریں گے
            depth = 1.0  # پلیس ہولڈر - میٹر
            # کیمرہ میٹرکس کا استعمال کرتے ہوئے 2D پوائنٹ کو 3D میں تبدیل کریں
            x_2d, y_2d = det.center_2d
            x_3d = (x_2d - self.camera_matrix[0, 2]) * depth / self.camera_matrix[0, 0]
            y_3d = (y_2d - self.camera_matrix[1, 2]) * depth / self.camera_matrix[1, 1]
            z_3d = depth

            updated_det = DetectedObject(
                id=det.id,
                class_name=det.class_name,
                confidence=det.confidence,
                bbox=det.bbox,
                center_2d=det.center_2d,
                center_3d=(x_3d, y_3d, z_3d)
            )
            updated_detections.append(updated_det)

        return updated_detections

    def estimate_poses(self, detections: List[DetectedObject], image) -> List[DetectedObject]:
        """
        پتہ چلنے والی اشیاء کے لیے 6-DOF پوز اسٹیمیٹ کریں۔
        """
        # یہ ایک سادہ نفاذ ہے
        # عمل میں، آپ خصوصی پوز اسٹیمیشن ماڈلز استعمال کریں گے
        updated_detections = []
        for det in detections:
            # جمہوریت کے لیے، ہم ایک سادہ پوز اسٹیمیٹ شامل کریں گے
            # حقیقی نفاذ میں، PVNet، Pix2Pose، وغیرہ جیسے ماڈلز استعمال کریں
            pose_3d = {
                'x': det.center_3d[0] if det.center_3d else 0.0,
                'y': det.center_3d[1] if det.center_3d else 0.0,
                'z': det.center_3d[2] if det.center_3d else 0.0,
                'roll': 0.0,  # پلیس ہولڈر
                'pitch': 0.0,  # پلیس ہولڈر
                'yaw': 0.0     # پلیس ہولڈر
            }

            updated_det = DetectedObject(
                id=det.id,
                class_name=det.class_name,
                confidence=det.confidence,
                bbox=det.bbox,
                center_2d=det.center_2d,
                center_3d=det.center_3d,
                pose_3d=pose_3d
            )
            updated_detections.append(updated_det)

        return updated_detections

    def understand_scene(self, detections: List[DetectedObject], image) -> Dict[str, Any]:
        """
        اشیاء کے پتے کی بنیاد پر منظر کی سمجھ کریں۔
        """
        scene_description = {
            'objects': [],
            'spatial_relationships': [],
            'functional_context': [],
            'navigation_space': self.estimate_navigation_space(image)
        }

        # شے کی معلومات شامل کریں
        for det in detections:
            obj_info = {
                'id': det.id,
                'class': det.class_name,
                'confidence': det.confidence,
                'position_3d': det.center_3d,
                'pose': det.pose_3d
            }
            scene_description['objects'].append(obj_info)

        # جگہی تعلقات کا تجزیہ کریں
        for i, obj1 in enumerate(detections):
            for j, obj2 in enumerate(detections[i+1:], i+1):
                if obj1.center_3d and obj2.center_3d:
                    rel = self.analyze_spatial_relationship(obj1, obj2)
                    scene_description['spatial_relationships'].append(rel)

        return scene_description

    def analyze_spatial_relationship(self, obj1: DetectedObject, obj2: DetectedObject) -> Dict[str, Any]:
        """
        دو اشیاء کے درمیان جگہی تعلق کا تجزیہ کریں۔
        """
        if not obj1.center_3d or not obj2.center_3d:
            return {}

        x1, y1, z1 = obj1.center_3d
        x2, y2, z2 = obj2.center_3d

        # ریلیٹیو پوزیشن کا حساب لگائیں
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        distance = np.sqrt(dx**2 + dy**2 + dz**2)

        # جگہی تعلق کا تعین کریں
        relationship = "قریب"
        if distance > 1.0:
            relationship = "دور"
        elif distance > 0.5:
            relationship = "درمیانہ"

        # ہدایت کا تعین کریں
        direction = "ایک ہی جگہ"
        if abs(dx) > abs(dy) and abs(dx) > abs(dz):
            direction = "بائیں/دائیں" if dx < 0 else "دائیں/بائیں"
        elif abs(dy) > abs(dz):
            direction = "سامنے/پیچھے" if dy < 0 else "پیچھے/سامنے"
        else:
            direction = "اوپر/نیچے" if dz < 0 else "نیچے/اوپر"

        return {
            'object1': obj1.id,
            'object2': obj2.id,
            'relationship': relationship,
            'direction': direction,
            'distance': distance,
            'relative_position': {'dx': dx, 'dy': dy, 'dz': dz}
        }

    def estimate_navigation_space(self, image) -> Dict[str, Any]:
        """
        تصویر سے نیوی گیشن سپیس کا تخمینہ لگائیں (سادہ)۔
        """
        # یہ ایک پلیس ہولڈر نفاذ ہے
        # عمل میں، ڈیپتھ معلومات اور رکاوٹ کا پتہ لگانے کا استعمال کریں
        height, width = image.shape[:2]
        return {
            'center_x': width // 2,
            'center_y': height // 2,
            'traversable_regions': [{'x': 0, 'y': 0, 'width': width, 'height': height}],
            'obstacle_regions': []
        }

    def publish_detections(self, detections: List[DetectedObject]):
        """
        ROS 2 ٹاپک پر اشیاء کے پتہ لگانے کے نتائج شائع کریں۔
        """
        detection_data = []
        for det in detections:
            det_dict = {
                'id': det.id,
                'class_name': det.class_name,
                'confidence': det.confidence,
                'bbox': {
                    'x': det.bbox[0],
                    'y': det.bbox[1],
                    'width': det.bbox[2],
                    'height': det.bbox[3]
                },
                'center_2d': {
                    'x': det.center_2d[0],
                    'y': det.center_2d[1]
                }
            }
            if det.center_3d:
                det_dict['center_3d'] = {
                    'x': det.center_3d[0],
                    'y': det.center_3d[1],
                    'z': det.center_3d[2]
                }
            if det.pose_3d:
                det_dict['pose_3d'] = det.pose_3d

            detection_data.append(det_dict)

        msg = String()
        msg.data = json.dumps({
            'timestamp': self.get_clock().now().to_msg().sec,
            'detections': detection_data
        })
        self.detections_publisher.publish(msg)

    def publish_scene_description(self, scene_description: Dict[str, Any]):
        """
        منظر کی سمجھ کے نتائج شائع کریں۔
        """
        msg = String()
        msg.data = json.dumps({
            'timestamp': self.get_clock().now().to_msg().sec,
            'scene': scene_description
        })
        self.scene_publisher.publish(msg)

    def destroy_node(self):
        """
        نوڈ کو تباہ کرنے سے پہلے وسائل صاف کریں۔
        """
        self.shutdown_requested = True
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """
    وژن پروسیسنگ نوڈ چلانے کے لیے مرکزی فنکشن۔
    """
    rclpy.init(args=args)

    vision_node = VisionProcessor()

    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        vision_node.get_logger().info('صارف کی طرف سے مداخلت، بند کیا جا رہا ہے...')
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## کوڈ کی مثالیں

### وژن-زبان گراؤنڈنگ نفاذ

```python
import numpy as np
import cv2
from typing import Dict, List, Any, Optional
import re


class VisionLanguageGrounding:
    """
    وژنل کمانڈز کو بصری ادراک میں زمین میں اتارنے کا سسٹم۔
    """

    def __init__(self, vision_processor: VisionProcessor):
        self.vision_processor = vision_processor
        self.object_reference_map = {}

    def ground_language_command(self, command: str, detections: List[DetectedObject]) -> Dict[str, Any]:
        """
        بصری منظر میں ایک زبانی کمانڈ کو زمین میں اتاریں۔
        """
        # کمانڈ کو اشیاء کے حوالہ جات اور جگہی معلومات کے لیے پارس کریں
        parsed_command = self.parse_language_command(command)

        # منظر میں متعلقہ اشیاء تلاش کریں
        relevant_objects = self.find_relevant_objects(parsed_command, detections)

        # جگہی تعلقات کو حل کریں
        resolved_targets = self.resolve_spatial_relationships(parsed_command, relevant_objects, detections)

        return {
            'command': command,
            'parsed_command': parsed_command,
            'relevant_objects': [obj.id for obj in relevant_objects],
            'targets': resolved_targets,
            'action_required': self.determine_action_type(command)
        }

    def parse_language_command(self, command: str) -> Dict[str, Any]:
        """
        اشیاء کے حوالہ جات اور جگہی معلومات نکالنے کے لیے ایک زبانی کمانڈ کو پارس کریں۔
        """
        command_lower = command.lower()

        # اشیاء کے حوالہ جات نکالیں
        object_patterns = [
            r'the (\w+) cup', r'a (\w+) cup', r'(\w+) cup',
            r'the (\w+) box', r'a (\w+) box', r'(\w+) box',
            r'the (\w+) bottle', r'a (\w+) bottle', r'(\w+) bottle',
            r'the (\w+)', r'a (\w+)', r'(\w+)',
        ]

        objects = []
        for pattern in object_patterns:
            matches = re.findall(pattern, command_lower)
            objects.extend(matches)

        # جگہی حوالہ جات نکالیں
        spatial_refs = []
        if 'left' in command_lower:
            spatial_refs.append('left')
        if 'right' in command_lower:
            spatial_refs.append('right')
        if 'front' in command_lower or 'in front' in command_lower:
            spatial_refs.append('front')
        if 'back' in command_lower or 'behind' in command_lower:
            spatial_refs.append('back')
        if 'near' in command_lower or 'close to' in command_lower:
            spatial_refs.append('near')
        if 'far' in command_lower or 'away from' in command_lower:
            spatial_refs.append('far')

        # رنگ نکالیں
        color_patterns = [
            r'(\w+) cup', r'(\w+) box', r'(\w+) bottle', r'(\w+) object'
        ]
        colors = []
        for pattern in color_patterns:
            matches = re.findall(pattern, command_lower)
            # اصل رنگوں کے لیے فلٹر کریں
            for match in matches:
                if match in ['red', 'blue', 'green', 'yellow', 'black', 'white', 'gray', 'orange', 'purple', 'pink']:
                    colors.append(match)

        return {
            'objects': list(set(objects)),
            'spatial_references': spatial_refs,
            'colors': list(set(colors)),
            'original_command': command
        }

    def find_relevant_objects(self, parsed_command: Dict[str, Any],
                            detections: List[DetectedObject]) -> List[DetectedObject]:
        """
        منظر میں وہ اشیاء تلاش کریں جو کمانڈ کی تفصیل سے مماثلت رکھتی ہیں۔
        """
        relevant_objects = []

        for det in detections:
            # چیک کریں کہ آیا شے کا کلاس کمانڈ سے مماثلت رکھتا ہے
            object_match = False
            for obj_type in parsed_command['objects']:
                if obj_type in det.class_name.lower():
                    object_match = True
                    break

            # چیک کریں کہ آیا رنگ کمانڈ سے مماثلت رکھتا ہے
            color_match = False
            if parsed_command['colors']:
                for color in parsed_command['colors']:
                    if color in det.class_name.lower():  # عمل میں، یہ الگ رنگ کا پتہ لگانے سے آئے گا
                        color_match = True
                        break
            else:
                color_match = True  # کوئی رنگ متعین نہیں، تو پہلے سے مماثلت

            if object_match and color_match:
                relevant_objects.append(det)

        return relevant_objects

    def resolve_spatial_relationships(self, parsed_command: Dict[str, Any],
                                    relevant_objects: List[DetectedObject],
                                    all_detections: List[DetectedObject]) -> List[Dict[str, Any]]:
        """
        مخصوص ہدف کی اشیاء کی شناخت کے لیے جگہی تعلقات کو حل کریں۔
        """
        if not relevant_objects:
            return []

        # اگر صرف ایک شے مماثلت رکھتی ہے، اسے لوٹائیں
        if len(relevant_objects) == 1:
            obj = relevant_objects[0]
            return [{
                'id': obj.id,
                'class_name': obj.class_name,
                'position_3d': obj.center_3d,
                'confidence': obj.confidence,
                'spatial_resolution': 'single_object'
            }]

        # اگر متعدد اشیاء مماثلت رکھتی ہیں، تضاد دور کرنے کے لیے جگہی تعلقات استعمال کریں
        targets = []
        for spatial_ref in parsed_command['spatial_references']:
            if spatial_ref == 'left':
                # سب سے بائیں والی شے تلاش کریں
                leftmost = min(relevant_objects, key=lambda x: x.center_2d[0] if x.center_2d else float('inf'))
                targets.append({
                    'id': leftmost.id,
                    'class_name': leftmost.class_name,
                    'position_3d': leftmost.center_3d,
                    'confidence': leftmost.confidence,
                    'spatial_resolution': f'{spatial_ref}_most'
                })
            elif spatial_ref == 'right':
                # سب سے دائیں والی شے تلاش کریں
                rightmost = max(relevant_objects, key=lambda x: x.center_2d[0] if x.center_2d else float('-inf'))
                targets.append({
                    'id': rightmost.id,
                    'class_name': rightmost.class_name,
                    'position_3d': rightmost.center_3d,
                    'confidence': rightmost.confidence,
                    'spatial_resolution': f'{spatial_ref}_most'
                })
            elif spatial_ref == 'front':
                # سب سے سامنے والی شے تلاش کریں (z-سمت میں قریب ترین)
                frontmost = min(relevant_objects, key=lambda x: x.center_3d[2] if x.center_3d else float('inf'))
                targets.append({
                    'id': frontmost.id,
                    'class_name': frontmost.class_name,
                    'position_3d': frontmost.center_3d,
                    'confidence': frontmost.confidence,
                    'spatial_resolution': f'{spatial_ref}_most'
                })
            elif spatial_ref == 'back':
                # سب سے پیچھے والی شے تلاش کریں (z-سمت میں دور ترین)
                backmost = max(relevant_objects, key=lambda x: x.center_3d[2] if x.center_3d else float('-inf'))
                targets.append({
                    'id': backmost.id,
                    'class_name': backmost.class_name,
                    'position_3d': backmost.center_3d,
                    'confidence': backmost.confidence,
                    'spatial_resolution': f'{spatial_ref}_most'
                })

        # اگر کوئی جگہی حوالہ استعمال نہیں کیا گیا، تمام متعلقہ اشیاء لوٹائیں
        if not targets and relevant_objects:
            for obj in relevant_objects:
                targets.append({
                    'id': obj.id,
                    'class_name': obj.class_name,
                    'position_3d': obj.center_3d,
                    'confidence': obj.confidence,
                    'spatial_resolution': 'all_relevant'
                })

        return targets

    def determine_action_type(self, command: str) -> str:
        """
        کمانڈ سے واضح ہونے والی ایکشن کی قسم کا تعین کریں۔
        """
        command_lower = command.lower()

        if any(word in command_lower for word in ['pick', 'grasp', 'grab', 'take']):
            return 'manipulation_pick'
        elif any(word in command_lower for word in ['place', 'put', 'set', 'drop']):
            return 'manipulation_place'
        elif any(word in command_lower for word in ['move', 'go', 'navigate', 'walk', 'drive']):
            return 'navigation'
        elif any(word in command_lower for word in ['find', 'locate', 'look', 'search']):
            return 'perception'
        else:
            return 'unknown'


# مثال کا استعمال
def create_vision_language_system():
    """
    ایک وژن-زبان گراؤنڈنگ سسٹم تخلیق کریں اور لوٹائیں۔
    """
    # عمل میں، آپ کے پاس ایک وژن پروسیسر انسٹنس ہوگا
    # اس مثال کے لیے، ہم ایک پلیس ہولڈر تخلیق کریں گے
    class PlaceholderVisionProcessor:
        pass

    vision_processor = PlaceholderVisionProcessor()
    grounding_system = VisionLanguageGrounding(vision_processor)
    return grounding_system
```

## ڈائریم (متن-مبنی)

### وژن پروسیسنگ پائپ لائن

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                    تصویر کا حصول                               │
                    │                  (کیمرہ، سینسرز)                             │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────▼───────────────────────────────────┐
                    │                  تصویر پری پروسیسنگ                           │
                    │              (ریکٹیفکیشن، ایکوائرنمنٹ)                      │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────▼───────────────────────────────────┐
                    │                   اشیاء کا پتہ لگانا                          │
                    │                (YOLO، Detectron2، وغیرہ)                     │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────┼───────────────────────────────────┐
                    │                         │                                   │
        ┌───────────▼──────────┐    ┌────────▼────────┐    ┌─────────────────────▼─────────────┐
        │   پتہ چلنے والی       │    │  3D پوزیشن      │    │      پوز                         │
        │   اشیاء (کلاسز،      │    │  اسٹیمیشن       │    │  اسٹیمیشن                       │
        │   بی بی اوز)          │    │                 │    │                                   │
        │                      │    │ • ڈیپتھ         │    │ • 6-DOF پوزز                     │
        │ • کلاس لیبلز         │    │ • کوآرڈینیٹ    │    │ • اورینٹیشن                     │
        │ • کمپلیمنس           │    │   ٹرانسفارمیشن │    │ • غیر یقینی                     │
        │ • 2D باؤنڈنگ        │    │ • اسکیل          │    │   اسٹیمیشن                      │
        │   باکسز              │    │   اسٹیمیشن     │    └─────────────────────────────────┘
        └──────────────────────┘    │                 │
                                  └─────────────────┘
```

### وژن-زبان گراؤنڈنگ عمل

```
قدرتی زبان کا کمانڈ ──► [زبانی پارسر] ──► [اشیاء کے حوالہ کی حل پذیری]
       │                           │                         │
       │                           ▼                         ▼
       │                   [جگہی ریلیشن شپ]     [بصری شے کا میچنگ]
       │                   [ایکسٹریکٹر]        [فیچر کمپیریزن]
       │                           │                         │
       └───────────────────────────┼─────────────────────────┼─────────► زمین میں اتارا ہدف
                                   ▼                         ▼
                           [منظر کا متن]            [جگہی منطق]
                           [انٹیگریشن]            [رکاوٹ ایپلیکیشن]
```

## عام مسائل

### 1. لائٹنگ کی حساسیت
**مسئلہ**: مختلف لائٹنگ کی حالت میں وژن سسٹم خراب کام کرتے ہیں۔
**حل**: اڈاپٹیو پری پروسیسنگ نافذ کریں اور ایلومنیشن-ان ویرینٹ فیچرز استعمال کریں۔

### 2. اوکلیوژن ہینڈلنگ
**مسئلہ**: اشیاء جزوی طور پر چھپی ہو سکتی ہیں، جس کے نتیجے میں نامکمل پتہ لگانا ہوتا ہے۔
**حل**: متعدد نظروں کا فیوژن استعمال کریں اور متن-آگاہ تکمیل الگوری دھم استعمال کریں۔

### 3. حقیقی وقت کی کارکردگی
**مسئلہ**: پیچیدہ وژن الگوری دھم حقیقی وقت کی ضروریات پوری نہیں کر سکتے۔
**حل**: ایج ڈیپلومنٹ کے لیے ماڈلز کو بہتر بنائیں اور موثر آرکیٹیکچر استعمال کریں۔

### 4. کیلیبریشن کی انحصار
**مسئلہ**: 3D پوز اسٹیمیشن براہ راست کیمرہ کیلیبریشن کی درستگی پر منحصر ہے۔
**حل**: خودکار کیلیبریشن روتینز اور توثیق چیکس نافذ کریں۔

### 5. زبان کا ابیمبگویٹی
**مسئلہ**: بصری متن کے بغیر قدرتی زبان کے کمانڈز مبہم ہو سکتے ہیں۔
**حل**: مسلسل وضاحت کے ذریعے تضاد دور کرنا نافذ کریں۔

## چیک پوائنٹس

### سمجھ کا چیک 1: وژن تصورات
- اشیاء کا پتہ لگانا اور پوز اسٹیمیشن میں کیا فرق ہے؟
- کیمرہ کیلیبریشن 3D دوبارہ تعمیر کو کیسے فعال کرتا ہے؟
- وژن-زبان گراؤنڈنگ میں چیلنج کیا ہیں؟

### سمجھ کا چیک 2: نفاذ
- آپ وژن سسٹم کو باہر کے ماحول کے لیے کیسے تبدیل کریں گے؟
- کون سے اضافی سینسر ادراک کی درستگی کو بہتر بنائیں گے؟
- آپ ایمبیڈڈ روبوٹک پلیٹ فارم کے لیے سسٹم کو کیسے بہتر بنائیں گے؟

### اطلاق کا چیک: وژن انضمام
- آپ سسٹم کو شفاف اشیاء کو ہینڈل کرنے کے لیے کیسے بڑھائیں گے؟
- وژن-مبنی نیوی گیشن کے لیے آپ کون سے سیفٹی اقدامات نافذ کریں گے؟
- آپ متحرک ماحول کو متحرک اشیاء کے ساتھ کیسے ہینڈل کریں گے؟

## حوالہ جات

1. Redmon, J., & Farhadi, A. (2018). "YOLOv3: ایک اضافی بہتری۔" *arXiv پری پرنٹ arXiv:1804.02767*۔

2. Carion, N., وغیرہ (2020). "ٹرانسفارمروں کے ساتھ اینڈ-ٹو-اینڈ اشیاء کا پتہ لگانا۔" *یورپی کانفرنس آن کمپیوٹر وژن (ECCV)*۔

3. Facebook AI. (2020). "Detectron2." دستیاب: https://github.com/facebookresearch/detectron2

4. OpenCV ٹیم. (2023). "OpenCV: اوپن سورس کمپیوٹر وژن لائبریری۔" دستیاب: https://opencv.org/

5. TorchVision ٹیم. (2023). "TorchVision: کمپیوٹر وژن کے لیے ڈیٹا سیٹس، ٹرانسفارم اور ماڈلز۔" دستیاب: https://pytorch.org/vision/

6. Chen, D., وغیرہ (2019). "ریل ورلڈ سنگل-امیج سوپر-ریزولوشن کی طرف: ایک نیا بینچ مارک اور ایک نیا ماڈل۔" *IEEE بین الاقوامی کانفرنس آن کمپیوٹر وژن (ICCV)*۔

7. Martinez-Cortes, M., وغیرہ (2018). "حقیقی ماحول میں سیمینٹک نیوی گیشن کے لیے کمپیوٹر وژن۔" *IEEE ٹرانزیکشن آن روبوٹکس*، 34(3)، 742-754۔

8. Hermans, M., وغیرہ (2014). "کئی روبوٹ موڈلز کو فعال کرنے کے لیے زبان کو زمین میں اتارنا۔" *IEEE بین الاقوامی کانفرنس آن روبوٹکس اینڈ آٹومیشن (ICRA)*۔

9. Misra, I., وغیرہ (2018). "ریفورسمنٹ لرننگ کے ساتھ ہدایات اور بصری مشاہدات کو ایکشنز میں میپ کرنا۔" *روبوٹکس: سائنس اینڈ سسٹم (RSS)*۔

10. Chen, L. C., وغیرہ (2018). "سیمینٹک امیج سیگمینٹیشن کے لیے ایٹریس سیپریبل کنولوشن کے ساتھ انکوڈر-ڈیکوڈر۔" *یورپی کانفرنس آن کمپیوٹر وژن (ECCV)*۔