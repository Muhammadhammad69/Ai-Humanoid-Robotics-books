#!/usr/bin/env python3
"""
LLM-Driven Cognitive Planning Example

This script demonstrates how to implement LLM-based cognitive planning for humanoid robots.
It creates a system that takes high-level natural language commands and decomposes them
into executable ROS 2 actions using Large Language Models.

Author: Robotics Developer
Date: 2025-12-10
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import openai
import json
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum
import asyncio


class TaskType(Enum):
    """Enumeration of different task types for robotic planning."""
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    PERCEPTION = "perception"
    COMMUNICATION = "communication"
    WAIT = "wait"


@dataclass
class Task:
    """Represents a single task in the planning system."""
    id: str
    type: TaskType
    action: str
    parameters: Dict[str, Any]
    dependencies: List[str]
    priority: int = 1
    estimated_duration: float = 1.0  # in seconds


@dataclass
class Plan:
    """Represents a complete plan with multiple tasks."""
    id: str
    original_command: str
    tasks: List[Task]
    context: Dict[str, Any]
    estimated_duration: float = 0.0


class LLMCognitivePlanner(Node):
    """
    ROS 2 node that uses LLMs for cognitive planning in humanoid robots.
    """

    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # Initialize OpenAI client (in practice, configure with your API key)
        # self.openai_client = openai.AsyncOpenAI(api_key="your-api-key")

        # Publishers
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

        self.feedback_publisher = self.create_publisher(
            String,
            'planning_feedback',
            10
        )

        # Subscribers
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

        # Internal state
        self.current_environment = {}
        self.active_plans = {}
        self.task_counter = 0

        # Parameters
        self.declare_parameter('llm_model', 'gpt-4-turbo')
        self.declare_parameter('max_planning_retries', 3)
        self.declare_parameter('plan_validation_enabled', True)
        self.declare_parameter('min_confidence_threshold', 0.7)

        self.get_logger().info('LLM Cognitive Planner initialized')

    def command_callback(self, msg):
        """
        Callback for high-level commands from users or other systems.
        """
        command_text = msg.data
        self.get_logger().info(f'Received high-level command: {command_text}')

        # Generate plan based on command and current environment
        future = asyncio.run_coroutine_threadsafe(
            self.generate_plan_async(command_text, self.current_environment),
            asyncio.new_event_loop()
        )

        # For simplicity in this example, we'll use the synchronous version
        plan = self.generate_plan(command_text, self.current_environment)

        if plan:
            # Publish the plan
            plan_json = json.dumps({
                'plan_id': plan.id,
                'original_command': plan.original_command,
                'tasks': [
                    {
                        'id': task.id,
                        'type': task.type.value,
                        'action': task.action,
                        'parameters': task.parameters,
                        'dependencies': task.dependencies,
                        'priority': task.priority,
                        'estimated_duration': task.estimated_duration
                    } for task in plan.tasks
                ],
                'context': plan.context,
                'estimated_duration': plan.estimated_duration
            })

            plan_msg = String()
            plan_msg.data = plan_json
            self.plan_publisher.publish(plan_msg)

            feedback_msg = String()
            feedback_msg.data = f'Generated plan with {len(plan.tasks)} tasks for command: {command_text}'
            self.feedback_publisher.publish(feedback_msg)

            self.get_logger().info(f'Published plan with {len(plan.tasks)} tasks')
        else:
            feedback_msg = String()
            feedback_msg.data = f'Failed to generate plan for command: {command_text}'
            self.feedback_publisher.publish(feedback_msg)
            self.get_logger().error('Failed to generate plan for command')

    def environment_callback(self, msg):
        """
        Update the planner's understanding of the environment.
        """
        try:
            self.current_environment = json.loads(msg.data)
            self.get_logger().debug('Environment state updated')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid environment state JSON')

    def generate_plan(self, command: str, environment: Dict[str, Any]) -> Optional[Plan]:
        """
        Generate a plan using LLM-based task decomposition.
        """
        try:
            # Create a detailed prompt for the LLM
            prompt = self.create_planning_prompt(command, environment)

            # Simulate LLM call (in practice, use actual LLM API)
            # For this example, we'll use a simulated response
            plan_data = self.simulate_llm_planning(command, environment)

            # Validate the plan
            if self.get_parameter('plan_validation_enabled').value:
                if not self.validate_plan(plan_data, environment):
                    self.get_logger().warn('Generated plan failed validation')
                    return None

            # Create Plan object from planning response
            tasks = []
            for task_data in plan_data['tasks']:
                task = Task(
                    id=task_data['id'],
                    type=TaskType(task_data['type']),
                    action=task_data['action'],
                    parameters=task_data['parameters'],
                    dependencies=task_data.get('dependencies', []),
                    priority=task_data.get('priority', 1),
                    estimated_duration=task_data.get('estimated_duration', 1.0)
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
            self.get_logger().error(f'Error generating plan: {e}')
            return None

    async def generate_plan_async(self, command: str, environment: Dict[str, Any]) -> Optional[Plan]:
        """
        Asynchronously generate a plan using LLM-based task decomposition.
        """
        try:
            # Create a detailed prompt for the LLM
            prompt = self.create_planning_prompt(command, environment)

            # Call the LLM asynchronously
            # response = await self.openai_client.chat.completions.create(
            #     model=self.get_parameter('llm_model').value,
            #     messages=[{"role": "user", "content": prompt}],
            #     temperature=0.1,
            #     response_format={"type": "json_object"}
            # )
            #
            # # Parse the LLM response
            # llm_response = response.choices[0].message.content
            # plan_data = json.loads(llm_response)

            # For demonstration, use simulated response
            plan_data = self.simulate_llm_planning(command, environment)

            # Validate the plan
            if self.get_parameter('plan_validation_enabled').value:
                if not self.validate_plan(plan_data, environment):
                    self.get_logger().warn('Generated plan failed validation')
                    return None

            # Create Plan object from planning response
            tasks = []
            for task_data in plan_data['tasks']:
                task = Task(
                    id=task_data['id'],
                    type=TaskType(task_data['type']),
                    action=task_data['action'],
                    parameters=task_data['parameters'],
                    dependencies=task_data.get('dependencies', []),
                    priority=task_data.get('priority', 1),
                    estimated_duration=task_data.get('estimated_duration', 1.0)
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
            self.get_logger().error(f'Error generating plan asynchronously: {e}')
            return None

    def create_planning_prompt(self, command: str, environment: Dict[str, Any]) -> str:
        """
        Create a detailed prompt for the LLM to generate a plan.
        """
        return f"""
        You are an advanced cognitive planning system for a humanoid robot. Your task is to decompose high-level natural language commands into executable robotic actions.

        Environmental Context:
        {json.dumps(environment, indent=2)}

        Command to Execute:
        "{command}"

        Please decompose this command into a detailed sequence of robotic actions following these requirements:

        1. Each task should be specific, actionable, and executable by the robot
        2. Consider all environmental constraints and safety requirements
        3. Include perception tasks when the robot needs to observe/understand its environment
        4. Include navigation tasks for movement between locations
        5. Include manipulation tasks for object interaction
        6. Specify dependencies between tasks where order matters
        7. Assign priorities to tasks (1=normal, 2=high, 3=critical)
        8. Estimate duration for each task in seconds

        Respond in strict JSON format:
        {{
            "plan_id": "unique_plan_identifier",
            "tasks": [
                {{
                    "id": "task_unique_id",
                    "type": "navigation|manipulation|perception|communication|wait",
                    "action": "specific_action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "dependencies": ["task_id_1", "task_id_2"],
                    "priority": 1,
                    "estimated_duration": 5.0
                }}
            ],
            "context": {{"additional_context": "information", "command_interpretation": "how_command_was_interpreted"}},
            "estimated_duration": 120.0
        }}

        Task Type Guidelines:
        - navigation: Moving the robot to specific locations
        - manipulation: Interacting with objects (grasping, placing, etc.)
        - perception: Sensing and understanding the environment
        - communication: Providing feedback or requesting assistance
        - wait: Pausing execution for specified duration

        Example for "Bring me the red cup from the table":
        {{
            "plan_id": "bring_red_cup_001",
            "tasks": [
                {{
                    "id": "locate_cup_001",
                    "type": "perception",
                    "action": "detect_object",
                    "parameters": {{"object_type": "cup", "color": "red"}},
                    "dependencies": [],
                    "priority": 2,
                    "estimated_duration": 3.0
                }},
                {{
                    "id": "navigate_to_cup_001",
                    "type": "navigation",
                    "action": "move_to_location",
                    "parameters": {{"x": 1.5, "y": 2.0, "theta": 0.0}},
                    "dependencies": ["locate_cup_001"],
                    "priority": 2,
                    "estimated_duration": 10.0
                }},
                {{
                    "id": "grasp_cup_001",
                    "type": "manipulation",
                    "action": "grasp_object",
                    "parameters": {{"object_id": "red_cup_001"}},
                    "dependencies": ["navigate_to_cup_001"],
                    "priority": 2,
                    "estimated_duration": 5.0
                }}
            ],
            "context": {{"original_command": "Bring me the red cup from the table", "target_object": "red_cup", "delivery_location": "user_position"}},
            "estimated_duration": 30.0
        }}
        """

    def simulate_llm_planning(self, command: str, environment: Dict[str, Any]) -> Dict[str, Any]:
        """
        Simulate LLM planning for demonstration purposes.
        In a real implementation, this would be replaced with actual LLM call.
        """
        self.task_counter += 1
        command_lower = command.lower()

        # Handle different command types
        if "clean the room" in command_lower:
            return {
                "plan_id": f"clean_room_{self.task_counter}",
                "tasks": [
                    {
                        "id": f"scan_{self.task_counter}_1",
                        "type": "perception",
                        "action": "scan_environment",
                        "parameters": {},
                        "dependencies": [],
                        "priority": 1,
                        "estimated_duration": 5.0
                    },
                    {
                        "id": f"locate_trash_{self.task_counter}_2",
                        "type": "perception",
                        "action": "detect_trash_objects",
                        "parameters": {},
                        "dependencies": [f"scan_{self.task_counter}_1"],
                        "priority": 2,
                        "estimated_duration": 8.0
                    },
                    {
                        "id": f"navigate_bin_{self.task_counter}_3",
                        "type": "navigation",
                        "action": "navigate_to_location",
                        "parameters": {"x": 0.5, "y": 1.0, "theta": 0.0},
                        "dependencies": [],
                        "priority": 1,
                        "estimated_duration": 12.0
                    },
                    {
                        "id": f"pickup_trash_{self.task_counter}_4",
                        "type": "manipulation",
                        "action": "pickup_object",
                        "parameters": {"object_id": "trash_1"},
                        "dependencies": [f"locate_trash_{self.task_counter}_2"],
                        "priority": 2,
                        "estimated_duration": 6.0
                    },
                    {
                        "id": f"dispose_trash_{self.task_counter}_5",
                        "type": "manipulation",
                        "action": "dispose_object",
                        "parameters": {"object_id": "trash_1", "destination": "waste_bin"},
                        "dependencies": [f"pickup_trash_{self.task_counter}_4", f"navigate_bin_{self.task_counter}_3"],
                        "priority": 2,
                        "estimated_duration": 4.0
                    }
                ],
                "context": {"original_command": command, "environment": environment},
                "estimated_duration": 120.0
            }

        elif "bring me" in command_lower or "get me" in command_lower:
            # Extract object from command
            import re
            object_match = re.search(r"bring me (?:the )?(.+)|get me (?:the )?(.+)", command_lower)
            object_name = "object"
            if object_match:
                object_name = object_match.group(1) or object_match.group(2) or "object"

            return {
                "plan_id": f"fetch_{object_name.replace(' ', '_')}_{self.task_counter}",
                "tasks": [
                    {
                        "id": f"locate_{self.task_counter}_1",
                        "type": "perception",
                        "action": "locate_object",
                        "parameters": {"object_type": object_name},
                        "dependencies": [],
                        "priority": 2,
                        "estimated_duration": 7.0
                    },
                    {
                        "id": f"navigate_{self.task_counter}_2",
                        "type": "navigation",
                        "action": "navigate_to_object",
                        "parameters": {"object_type": object_name},
                        "dependencies": [f"locate_{self.task_counter}_1"],
                        "priority": 2,
                        "estimated_duration": 10.0
                    },
                    {
                        "id": f"grasp_{self.task_counter}_3",
                        "type": "manipulation",
                        "action": "grasp_object",
                        "parameters": {"object_type": object_name},
                        "dependencies": [f"navigate_{self.task_counter}_2"],
                        "priority": 2,
                        "estimated_duration": 5.0
                    },
                    {
                        "id": f"return_{self.task_counter}_4",
                        "type": "navigation",
                        "action": "navigate_to_user",
                        "parameters": {},
                        "dependencies": [f"grasp_{self.task_counter}_3"],
                        "priority": 2,
                        "estimated_duration": 8.0
                    },
                    {
                        "id": f"deliver_{self.task_counter}_5",
                        "type": "manipulation",
                        "action": "deliver_object",
                        "parameters": {"object_type": object_name},
                        "dependencies": [f"return_{self.task_counter}_4"],
                        "priority": 2,
                        "estimated_duration": 3.0
                    }
                ],
                "context": {"original_command": command, "target_object": object_name, "environment": environment},
                "estimated_duration": 60.0
            }

        elif "go to" in command_lower or "move to" in command_lower:
            # Extract destination from command
            import re
            dest_match = re.search(r"go to (.+)|move to (.+)", command_lower)
            destination = "specified location"
            if dest_match:
                destination = dest_match.group(1) or dest_match.group(2) or "specified location"

            return {
                "plan_id": f"navigate_to_{destination.replace(' ', '_')}_{self.task_counter}",
                "tasks": [
                    {
                        "id": f"find_path_{self.task_counter}_1",
                        "type": "perception",
                        "action": "plan_path",
                        "parameters": {"destination": destination},
                        "dependencies": [],
                        "priority": 1,
                        "estimated_duration": 2.0
                    },
                    {
                        "id": f"move_{self.task_counter}_2",
                        "type": "navigation",
                        "action": "move_to_destination",
                        "parameters": {"destination": destination},
                        "dependencies": [f"find_path_{self.task_counter}_1"],
                        "priority": 1,
                        "estimated_duration": 15.0
                    }
                ],
                "context": {"original_command": command, "destination": destination, "environment": environment},
                "estimated_duration": 20.0
            }

        else:
            # Default response for unrecognized commands
            return {
                "plan_id": f"unknown_command_{self.task_counter}",
                "tasks": [
                    {
                        "id": f"request_clarification_{self.task_counter}_1",
                        "type": "communication",
                        "action": "request_clarification",
                        "parameters": {"message": f"I don't understand how to execute: {command}"},
                        "dependencies": [],
                        "priority": 3,
                        "estimated_duration": 5.0
                    }
                ],
                "context": {"original_command": command, "environment": environment},
                "estimated_duration": 10.0
            }

    def validate_plan(self, plan_data: Dict[str, Any], environment: Dict[str, Any]) -> bool:
        """
        Validate the generated plan for safety and feasibility.
        """
        try:
            # Check if all required fields are present
            required_fields = ['plan_id', 'tasks', 'context']
            for field in required_fields:
                if field not in plan_data:
                    self.get_logger().error(f'Missing required field in plan: {field}')
                    return False

            # Validate each task
            for task in plan_data['tasks']:
                required_task_fields = ['id', 'type', 'action', 'parameters', 'dependencies']
                for field in required_task_fields:
                    if field not in task:
                        self.get_logger().error(f'Task {task.get("id", "unknown")} missing required field: {field}')
                        return False

                # Validate task type
                try:
                    TaskType(task['type'])
                except ValueError:
                    self.get_logger().error(f'Invalid task type in plan: {task["type"]}')
                    return False

                # Validate dependencies exist
                for dep_id in task.get('dependencies', []):
                    task_ids = [t['id'] for t in plan_data['tasks']]
                    if dep_id not in task_ids:
                        self.get_logger().warn(f'Dependency {dep_id} not found in plan for task {task["id"]}')

            # Additional safety validations could go here
            # For example, checking if navigation targets are in safe areas
            # or if manipulation actions are within robot capabilities

            return True

        except Exception as e:
            self.get_logger().error(f'Error validating plan: {e}')
            return False


def main(args=None):
    """
    Main function to run the LLM-based cognitive planner node.
    """
    rclpy.init(args=args)

    planner = LLMCognitivePlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Interrupted by user, shutting down...')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()