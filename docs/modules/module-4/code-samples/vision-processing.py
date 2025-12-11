#!/usr/bin/env python3
"""
Vision Processing Example for Humanoid Robots

This script demonstrates computer vision processing for humanoid robots,
including object detection, pose estimation, and scene understanding.
It integrates with ROS 2 for real-time robotic perception.

Author: Robotics Developer
Date: 2025-12-10
"""

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
    """Represents a detected object in the scene."""
    id: str
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x, y, width, height)
    center_2d: Tuple[int, int]       # 2D center in image coordinates
    center_3d: Optional[Tuple[float, float, float]] = None  # 3D center in world coordinates
    pose_3d: Optional[Dict[str, float]] = None  # 6-DOF pose if available


class RobotVisionProcessor(Node):
    """
    ROS 2 node for computer vision processing in humanoid robots.
    Handles object detection, pose estimation, and scene understanding.
    """

    def __init__(self):
        super().__init__('robot_vision_processor')

        # Initialize vision components
        self.bridge = CvBridge()

        # Load object detection model (using TorchVision's pre-trained model as example)
        self.detection_model = self.load_detection_model()
        self.detection_transform = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # Image processing queue
        self.image_queue = queue.Queue(maxsize=10)

        # Publishers
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

        self.feedback_publisher = self.create_publisher(
            String,
            'vision/feedback',
            10
        )

        # Subscribers
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

        # Internal state
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.latest_image = None
        self.processing_thread = None
        self.shutdown_requested = False
        self.object_counter = 0

        # Parameters
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('enable_pose_estimation', True)
        self.declare_parameter('enable_scene_understanding', True)
        self.declare_parameter('max_detection_objects', 20)
        self.declare_parameter('image_processing_rate', 10.0)  # Hz

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_images, daemon=True)
        self.processing_thread.start()

        self.get_logger().info('Robot Vision Processor initialized')

    def load_detection_model(self):
        """
        Load the object detection model.
        In practice, this could be YOLO, Detectron2, or other models.
        """
        try:
            # Load YOLOv5 model
            model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            model.eval()
            self.get_logger().info('YOLOv5 model loaded successfully')
            return model
        except Exception as e:
            self.get_logger().warn(f'Failed to load YOLOv5 model: {e}')
            # Fallback to a simple placeholder model
            return None

    def image_callback(self, msg):
        """
        Callback for incoming camera images.
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image

            # Add to processing queue
            if not self.image_queue.full():
                self.image_queue.put((msg.header.stamp, cv_image))
            else:
                # Drop oldest image if queue is full
                try:
                    self.image_queue.get_nowait()
                    self.image_queue.put((msg.header.stamp, cv_image))
                except queue.Empty:
                    pass

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def camera_info_callback(self, msg):
        """
        Callback for camera calibration information.
        """
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def vision_command_callback(self, msg):
        """
        Handle vision processing commands.
        """
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '')

            if command == 'detect_objects':
                self.get_logger().info('Object detection command received')
                # Trigger immediate processing if an image is available
                if self.latest_image is not None:
                    self.process_single_image(self.latest_image)
                else:
                    feedback_msg = String()
                    feedback_msg.data = 'No image available for processing'
                    self.feedback_publisher.publish(feedback_msg)

            elif command == 'estimate_poses':
                self.get_logger().info('Pose estimation command received')
                # Additional processing for pose estimation
                if self.latest_image is not None:
                    self.process_single_image(self.latest_image, estimate_poses=True)
                else:
                    feedback_msg = String()
                    feedback_msg.data = 'No image available for pose estimation'
                    self.feedback_publisher.publish(feedback_msg)

            elif command == 'understand_scene':
                self.get_logger().info('Scene understanding command received')
                if self.latest_image is not None:
                    self.process_single_image(self.latest_image, understand_scene=True)
                else:
                    feedback_msg = String()
                    feedback_msg.data = 'No image available for scene understanding'
                    self.feedback_publisher.publish(feedback_msg)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid vision command JSON')

    def process_images(self):
        """
        Continuously process images from the queue.
        """
        rate = self.get_parameter('image_processing_rate').value
        timer = self.create_rate(rate)

        while not self.shutdown_requested:
            try:
                # Get image from queue with timeout
                timestamp, image = self.image_queue.get(timeout=0.1)

                # Process the image
                self.process_single_image(image)

                # Control processing rate
                timer.sleep()

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in image processing loop: {e}')

    def process_single_image(self, image, estimate_poses=True, understand_scene=True):
        """
        Process a single image for object detection and other vision tasks.
        """
        try:
            # Perform object detection
            detections = self.detect_objects(image)

            # If camera info is available, compute 3D positions
            if self.camera_matrix is not None and self.distortion_coeffs is not None:
                detections = self.compute_3d_positions(detections, image)

            # Perform pose estimation if enabled
            if self.get_parameter('enable_pose_estimation').value and estimate_poses:
                detections = self.estimate_poses(detections, image)

            # Publish detection results
            if detections:
                self.publish_detections(detections)
                self.get_logger().info(f'Published {len(detections)} detections')

            # Perform scene understanding if enabled
            if self.get_parameter('enable_scene_understanding').value and understand_scene:
                scene_description = self.understand_scene(detections, image)
                self.publish_scene_description(scene_description)

        except Exception as e:
            self.get_logger().error(f'Error processing single image: {e}')

    def detect_objects(self, image) -> List[DetectedObject]:
        """
        Perform object detection on the image.
        """
        if self.detection_model is None:
            # Fallback: return no detections
            return []

        try:
            # Convert image for model input
            img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.detection_model(img_rgb)

            # Parse detection results
            detections = []
            for i, det in enumerate(results.xyxy[0]):  # xyxy format: [x1, y1, x2, y2, conf, class]
                x1, y1, x2, y2, conf, cls = det
                x1, y1, x2, y2, conf, cls = int(x1), int(y1), int(x2), int(y2), float(conf), int(cls)

                # Get class name
                class_names = self.detection_model.names if hasattr(self.detection_model, 'names') else {}
                class_name = class_names.get(cls, f'unknown_{cls}')

                # Check confidence threshold
                if conf >= self.get_parameter('detection_threshold').value:
                    width = x2 - x1
                    height = y2 - y1
                    center_2d = ((x1 + x2) // 2, (y1 + y2) // 2)

                    obj = DetectedObject(
                        id=f"obj_{self.object_counter}",
                        class_name=class_name,
                        confidence=conf,
                        bbox=(x1, y1, width, height),
                        center_2d=center_2d
                    )
                    detections.append(obj)
                    self.object_counter += 1

                    # Limit number of detections
                    if len(detections) >= self.get_parameter('max_detection_objects').value:
                        break

            return detections

        except Exception as e:
            self.get_logger().error(f'Error in object detection: {e}')
            return []

    def compute_3d_positions(self, detections: List[DetectedObject], image) -> List[DetectedObject]:
        """
        Compute 3D positions from 2D detections using camera calibration.
        """
        if not detections or self.camera_matrix is None:
            return detections

        updated_detections = []
        for det in detections:
            # Simple depth estimation (in practice, use stereo vision or depth sensor)
            # For this example, we'll use a simple inverse relationship to bounding box size
            # In real implementation, you'd use actual depth data
            bbox_width = det.bbox[2]
            # Simulate depth based on object size (larger objects appear closer)
            # This is a very simplified approach
            depth = max(0.1, 10.0 / (bbox_width + 1))  # Placeholder depth calculation

            # Convert 2D point to 3D using camera matrix
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
        Estimate 6-DOF poses for detected objects.
        """
        # This is a simplified implementation
        # In practice, you'd use specialized pose estimation models
        updated_detections = []
        for det in detections:
            # For demonstration, we'll add a simple pose estimate
            # In real implementation, use models like PVNet, Pix2Pose, etc.
            pose_3d = {
                'x': det.center_3d[0] if det.center_3d else 0.0,
                'y': det.center_3d[1] if det.center_3d else 0.0,
                'z': det.center_3d[2] if det.center_3d else 0.0,
                'roll': 0.0,  # Placeholder
                'pitch': 0.0,  # Placeholder
                'yaw': 0.0     # Placeholder
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
        Perform scene understanding based on object detections.
        """
        scene_description = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'objects': [],
            'spatial_relationships': [],
            'functional_context': [],
            'navigation_space': self.estimate_navigation_space(image),
            'object_count': len(detections)
        }

        # Add object information
        for det in detections:
            obj_info = {
                'id': det.id,
                'class': det.class_name,
                'confidence': det.confidence,
                'position_3d': det.center_3d,
                'pose': det.pose_3d,
                'bbox': {
                    'x': det.bbox[0],
                    'y': det.bbox[1],
                    'width': det.bbox[2],
                    'height': det.bbox[3]
                }
            }
            scene_description['objects'].append(obj_info)

        # Analyze spatial relationships
        for i, obj1 in enumerate(detections):
            for j, obj2 in enumerate(detections[i+1:], i+1):
                if obj1.center_3d and obj2.center_3d:
                    rel = self.analyze_spatial_relationship(obj1, obj2)
                    scene_description['spatial_relationships'].append(rel)

        # Identify functional contexts (simplified)
        scene_description['functional_context'] = self.identify_functional_contexts(detections)

        return scene_description

    def analyze_spatial_relationship(self, obj1: DetectedObject, obj2: DetectedObject) -> Dict[str, Any]:
        """
        Analyze spatial relationship between two objects.
        """
        if not obj1.center_3d or not obj2.center_3d:
            return {}

        x1, y1, z1 = obj1.center_3d
        x2, y2, z2 = obj2.center_3d

        # Calculate relative position
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        distance = np.sqrt(dx**2 + dy**2 + dz**2)

        # Determine spatial relationship
        relationship = "near"
        if distance > 1.0:
            relationship = "far"
        elif distance > 0.5:
            relationship = "medium"

        # Determine directional relationship
        direction = "same position"
        if abs(dx) > abs(dy) and abs(dx) > abs(dz):
            direction = "left/right" if dx < 0 else "right/left"
        elif abs(dy) > abs(dz):
            direction = "front/back" if dy < 0 else "back/front"
        else:
            direction = "above/below" if dz < 0 else "below/above"

        return {
            'object1': obj1.id,
            'object2': obj2.id,
            'relationship': relationship,
            'direction': direction,
            'distance': distance,
            'relative_position': {'dx': dx, 'dy': dy, 'dz': dz}
        }

    def identify_functional_contexts(self, detections: List[DetectedObject]) -> List[str]:
        """
        Identify functional contexts in the scene (e.g., dining area, workspace).
        """
        contexts = []

        # Check for dining area (table + chairs + cups/plates)
        has_table = any('table' in obj.class_name.lower() for obj in detections)
        has_chair = any('chair' in obj.class_name.lower() for obj in detections)
        has_dining_items = any(
            any(item in obj.class_name.lower() for item in ['cup', 'bottle', 'bowl', 'plate'])
            for obj in detections
        )

        if has_table and has_chair and has_dining_items:
            contexts.append('dining_area')

        # Check for workspace (table + laptop/keyboard)
        has_electronics = any(
            any(item in obj.class_name.lower() for item in ['laptop', 'keyboard', 'monitor', 'book'])
            for obj in detections
        )

        if has_table and has_electronics:
            contexts.append('workspace')

        # Check for storage area (cabinet/refrigerator + various objects)
        has_storage = any(
            any(item in obj.class_name.lower() for item in ['cabinet', 'refrigerator', 'shelf'])
            for obj in detections
        )

        if has_storage and len(detections) > 3:
            contexts.append('storage_area')

        return contexts

    def estimate_navigation_space(self, image) -> Dict[str, Any]:
        """
        Estimate navigable space from image (simplified approach).
        """
        height, width = image.shape[:2]

        # This is a very simplified approach
        # In practice, use depth information, obstacle detection, etc.
        return {
            'center_x': width // 2,
            'center_y': height // 2,
            'traversable_regions': [{'x': 0, 'y': 0, 'width': width, 'height': height}],
            'obstacle_regions': [],
            'safe_navigation_zone': {
                'x': width // 4,
                'y': height // 4,
                'width': width // 2,
                'height': height // 2
            }
        }

    def publish_detections(self, detections: List[DetectedObject]):
        """
        Publish object detection results to ROS 2 topic.
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
        Publish scene understanding results.
        """
        msg = String()
        msg.data = json.dumps(scene_description)
        self.scene_publisher.publish(msg)

    def destroy_node(self):
        """
        Clean up resources before node destruction.
        """
        self.shutdown_requested = True
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """
    Main function to run the robot vision processing node.
    """
    rclpy.init(args=args)

    vision_node = RobotVisionProcessor()

    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        vision_node.get_logger().info('Interrupted by user, shutting down...')
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()