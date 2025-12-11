#!/usr/bin/env python3
"""
Complete VLA Integration Example

This script demonstrates the complete integration of Vision-Language-Action systems
for humanoid robots. It combines voice processing, cognitive planning, and vision
processing into a unified autonomous system that responds to complex voice commands
with coordinated perception, reasoning, and action.

Author: Robotics Developer
Date: 2025-12-10
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, AudioData
from geometry_msgs.msg import Pose
import json
import threading
import queue
from typing import Dict, Any, Optional, List
import time
import uuid
import traceback


class VLAIntegrationSystem(Node):
    """
    Complete VLA Integration System combining voice, vision, and action components.
    """

    def __init__(self):
        super().__init__('vla_integration_system')

        # Initialize system components
        self.system_state = {
            'voice_commands': queue.Queue(),
            'vision_detections': queue.Queue(),
            'environment_state': {},
            'active_plan': None,
            'robot_pose': None,
            'system_status': 'idle',
            'last_command_time': time.time(),
            'error_recovery_mode': False,
            'components_ready': {
                'voice': False,
                'vision': False,
                'planning': False,
                'action': False
            }
        }

        # Publishers
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

        # Subscribers
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

        self.planning_subscriber = self.create_subscription(
            String,
            'robot_plan',
            self.planning_callback,
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

        # Control subscribers
        self.emergency_stop_subscriber = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10
        )

        # System control
        self.main_control_thread = None
        self.shutdown_requested = False
        self.component_monitor_thread = None

        # Parameters
        self.declare_parameter('max_command_age', 30.0)  # seconds
        self.declare_parameter('error_recovery_enabled', True)
        self.declare_parameter('system_heartbeat_interval', 1.0)  # seconds
        self.declare_parameter('execution_timeout', 300.0)  # seconds
        self.declare_parameter('component_check_interval', 5.0)  # seconds

        # Start main control thread
        self.main_control_thread = threading.Thread(target=self.main_control_loop, daemon=True)
        self.main_control_thread.start()

        # Start component monitoring thread
        self.component_monitor_thread = threading.Thread(target=self.monitor_components, daemon=True)
        self.component_monitor_thread.start()

        # Start heartbeat publisher
        self.heartbeat_timer = self.create_timer(
            self.get_parameter('system_heartbeat_interval').value,
            self.publish_system_heartbeat
        )

        self.get_logger().info('Complete VLA Integration System initialized')

    def voice_callback(self, msg):
        """
        Handle incoming voice commands.
        """
        try:
            command_data = json.loads(msg.data)
            self.system_state['voice_commands'].put(command_data)
            self.system_state['last_command_time'] = time.time()

            self.get_logger().info(f'Voice command received: {command_data.get("command", "unknown")}')

            # Update system status
            self.system_state['system_status'] = 'processing_command'
            self.publish_system_status()

            # Process the command
            self.process_voice_command(command_data)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid voice command JSON')
        except Exception as e:
            self.get_logger().error(f'Error in voice callback: {e}')

    def vision_callback(self, msg):
        """
        Handle incoming vision detections.
        """
        try:
            detection_data = json.loads(msg.data)
            self.system_state['vision_detections'].put(detection_data)

            # Update environment state with vision information
            self.update_environment_with_vision(detection_data)

            self.get_logger().debug(f'Vision detection received: {len(detection_data.get("detections", []))} objects')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid vision data JSON')
        except Exception as e:
            self.get_logger().error(f'Error in vision callback: {e}')

    def planning_callback(self, msg):
        """
        Handle incoming plans from the cognitive planner.
        """
        try:
            plan_data = json.loads(msg.data)
            self.system_state['active_plan'] = plan_data
            self.system_state['system_status'] = 'executing_plan'

            self.get_logger().info(f'Plan received: {plan_data.get("plan_id", "unknown")} with {len(plan_data.get("tasks", []))} tasks')

            # Execute the plan
            self.execute_plan(plan_data)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid plan data JSON')
        except Exception as e:
            self.get_logger().error(f'Error in planning callback: {e}')

    def environment_callback(self, msg):
        """
        Handle environment state updates.
        """
        try:
            env_data = json.loads(msg.data)
            self.system_state['environment_state'].update(env_data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid environment state JSON')
        except Exception as e:
            self.get_logger().error(f'Error in environment callback: {e}')

    def robot_pose_callback(self, msg):
        """
        Handle robot pose updates.
        """
        self.system_state['robot_pose'] = msg

    def emergency_stop_callback(self, msg):
        """
        Handle emergency stop commands.
        """
        if msg.data:
            self.get_logger().warn('Emergency stop activated!')
            self.system_state['system_status'] = 'emergency_stop'
            self.cancel_active_plan()
            self.publish_system_status()

    def process_voice_command(self, command_data: Dict[str, Any]):
        """
        Process a voice command through the complete VLA pipeline.
        """
        try:
            command_text = command_data.get('command', '')
            if not command_text:
                return

            self.get_logger().info(f'Processing voice command through VLA pipeline: {command_text}')

            # Step 1: Request vision processing for current scene
            vision_cmd = String()
            vision_cmd.data = json.dumps({
                'command': 'detect_objects',
                'context': command_data
            })
            self.vision_command_publisher.publish(vision_cmd)

            # Step 2: Send command to cognitive planner
            planning_cmd = String()
            planning_cmd.data = json.dumps({
                'command': command_text,
                'environment': self.system_state['environment_state'],
                'timestamp': command_data.get('timestamp', time.time())
            })
            self.planning_command_publisher.publish(planning_cmd)

            # Step 3: Wait for plan (in practice, this would be asynchronous)
            # For this example, we'll assume the planner will respond

        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')
            traceback.print_exc()

    def execute_plan(self, plan_data: Dict[str, Any]):
        """
        Execute a received plan with proper error handling and monitoring.
        """
        try:
            plan_id = plan_data.get('plan_id', str(uuid.uuid4()))
            self.get_logger().info(f'Executing plan: {plan_id}')

            # Publish each task in the plan sequentially
            for i, task in enumerate(plan_data.get('tasks', [])):
                if self.system_state['error_recovery_mode']:
                    self.get_logger().info('Plan execution stopped due to error recovery')
                    break

                # Check for emergency stop
                if self.system_state['system_status'] == 'emergency_stop':
                    self.get_logger().info('Plan execution stopped due to emergency stop')
                    break

                # Publish task for execution
                task_msg = String()
                task_msg.data = json.dumps({
                    'plan_id': plan_id,
                    'task_index': i,
                    'total_tasks': len(plan_data.get('tasks', [])),
                    'task': task,
                    'environment': self.system_state['environment_state'],
                    'timestamp': time.time()
                })
                self.command_publisher.publish(task_msg)

                self.get_logger().info(f'Published task {i+1}/{len(plan_data.get("tasks", []))}: {task.get("id", "unknown")}')

                # Wait for task completion (simplified - in practice, monitor actual completion)
                time.sleep(1.0)

            # Mark plan as complete
            self.system_state['active_plan'] = None
            self.system_state['system_status'] = 'idle'
            self.publish_system_status()

            feedback_msg = String()
            feedback_msg.data = json.dumps({
                'type': 'plan_completed',
                'plan_id': plan_id,
                'success': True,
                'message': f'Plan {plan_id} completed successfully'
            })
            self.feedback_publisher.publish(feedback_msg)

        except Exception as e:
            self.get_logger().error(f'Error executing plan: {e}')
            self.handle_execution_error(e, plan_data.get('plan_id'))

    def update_environment_with_vision(self, detection_data: Dict[str, Any]):
        """
        Update environment state with vision information.
        """
        try:
            # Update environment with object positions and scene information
            if 'detections' in detection_data:
                self.system_state['environment_state']['objects'] = detection_data['detections']

            if 'scene' in detection_data:
                self.system_state['environment_state']['scene'] = detection_data['scene']

            self.system_state['environment_state']['last_vision_update'] = time.time()

        except Exception as e:
            self.get_logger().error(f'Error updating environment with vision: {e}')

    def main_control_loop(self):
        """
        Main control loop for the VLA system.
        """
        while not self.shutdown_requested:
            try:
                # Process any queued voice commands
                self.process_queued_voice_commands()

                # Monitor active plans
                self.monitor_active_plan()

                # Check for system errors
                self.check_system_errors()

                # Handle error recovery if needed
                if self.system_state['error_recovery_mode']:
                    self.handle_error_recovery()

                # Small sleep to prevent busy waiting
                time.sleep(0.01)

            except Exception as e:
                self.get_logger().error(f'Error in main control loop: {e}')
                time.sleep(0.1)  # Brief pause on error

    def process_queued_voice_commands(self):
        """
        Process any voice commands in the queue.
        """
        while not self.system_state['voice_commands'].empty():
            try:
                command_data = self.system_state['voice_commands'].get_nowait()
                self.process_voice_command(command_data)
            except queue.Empty:
                break
            except Exception as e:
                self.get_logger().error(f'Error processing queued voice command: {e}')

    def monitor_active_plan(self):
        """
        Monitor the active plan for completion or errors.
        """
        if self.system_state['active_plan'] is None:
            return

        # Check if plan has exceeded timeout
        if (time.time() - self.system_state['last_command_time']) > self.get_parameter('execution_timeout').value:
            self.get_logger().warn('Active plan exceeded timeout, cancelling')
            self.cancel_active_plan()
            self.system_state['error_recovery_mode'] = True

    def cancel_active_plan(self):
        """
        Cancel the currently active plan.
        """
        if self.system_state['active_plan']:
            plan_id = self.system_state['active_plan'].get('plan_id', 'unknown')
            self.get_logger().info(f'Cancelling plan: {plan_id}')
            self.system_state['active_plan'] = None

        self.system_state['system_status'] = 'idle'
        self.publish_system_status()

    def check_system_errors(self):
        """
        Check for various system errors.
        """
        # Check for stale commands
        if (time.time() - self.system_state['last_command_time']) > self.get_parameter('max_command_age').value:
            if self.system_state['system_status'] == 'processing_command':
                self.get_logger().warn('Command is too old, resetting system state')
                self.system_state['system_status'] = 'idle'
                self.publish_system_status()

    def handle_error_recovery(self):
        """
        Handle system error recovery.
        """
        if not self.get_parameter('error_recovery_enabled').value:
            return

        self.get_logger().info('Entering error recovery mode')

        # Stop all current activities
        self.cancel_active_plan()

        # Clear command queue
        while not self.system_state['voice_commands'].empty():
            try:
                self.system_state['voice_commands'].get_nowait()
            except queue.Empty:
                break

        # Publish recovery status
        feedback_msg = String()
        feedback_msg.data = json.dumps({
            'type': 'error_recovery',
            'message': 'System in error recovery mode, awaiting new commands'
        })
        self.feedback_publisher.publish(feedback_msg)

        # Reset error recovery mode after a delay
        time.sleep(2.0)
        self.system_state['error_recovery_mode'] = False
        self.get_logger().info('Error recovery complete, returning to normal operation')

    def handle_execution_error(self, error: Exception, plan_id: Optional[str] = None):
        """
        Handle errors during plan execution.
        """
        self.get_logger().error(f'Execution error: {error}')
        if plan_id:
            self.get_logger().error(f'Error occurred during execution of plan: {plan_id}')

        # Cancel current plan
        self.cancel_active_plan()

        # Enter error recovery mode
        self.system_state['error_recovery_mode'] = True

        # Publish error feedback
        feedback_msg = String()
        feedback_msg.data = json.dumps({
            'type': 'execution_error',
            'plan_id': plan_id,
            'error': str(error),
            'message': f'Execution error: {str(error)}, entering recovery mode'
        })
        self.feedback_publisher.publish(feedback_msg)

    def monitor_components(self):
        """
        Monitor the status of various system components.
        """
        while not self.shutdown_requested:
            try:
                # Check if components are still responsive
                # This is a simplified check - in practice, you'd have heartbeats from each component
                current_time = time.time()

                # Update component readiness status (simplified)
                self.system_state['components_ready']['voice'] = True  # Assume voice component is ready
                self.system_state['components_ready']['vision'] = True  # Assume vision component is ready
                self.system_state['components_ready']['planning'] = True  # Assume planning component is ready
                self.system_state['components_ready']['action'] = True  # Assume action component is ready

                time.sleep(self.get_parameter('component_check_interval').value)

            except Exception as e:
                self.get_logger().error(f'Error in component monitoring: {e}')
                time.sleep(1.0)

    def publish_system_status(self):
        """
        Publish current system status.
        """
        try:
            status_msg = String()
            status_msg.data = json.dumps({
                'status': self.system_state['system_status'],
                'active_plan_id': self.system_state['active_plan']['plan_id'] if self.system_state['active_plan'] else None,
                'error_recovery_mode': self.system_state['error_recovery_mode'],
                'components_ready': self.system_state['components_ready'],
                'timestamp': time.time()
            })
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing system status: {e}')

    def publish_system_heartbeat(self):
        """
        Publish system heartbeat to indicate system is alive.
        """
        try:
            heartbeat_msg = String()
            heartbeat_msg.data = json.dumps({
                'system': 'complete_vla_integration_system',
                'status': self.system_state['system_status'],
                'timestamp': time.time(),
                'active_plan': self.system_state['active_plan'] is not None,
                'components_ready': self.system_state['components_ready']
            })
            self.status_publisher.publish(heartbeat_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing heartbeat: {e}')

    def destroy_node(self):
        """
        Clean up resources before node destruction.
        """
        self.shutdown_requested = True
        if self.main_control_thread:
            self.main_control_thread.join(timeout=2.0)
        if self.component_monitor_thread:
            self.component_monitor_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """
    Main function to run the complete VLA integration system.
    """
    rclpy.init(args=args)

    vla_system = VLAIntegrationSystem()

    try:
        rclpy.spin(vla_system)
    except KeyboardInterrupt:
        vla_system.get_logger().info('Interrupted by user, shutting down...')
    finally:
        vla_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()