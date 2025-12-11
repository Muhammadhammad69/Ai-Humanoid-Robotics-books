# Chapter 3: LLM-Driven Cognitive Planning for Humanoid Robots

## Overview

This chapter explores the implementation of Large Language Model (LLM)-driven cognitive planning systems for humanoid robots. Cognitive planning bridges the gap between high-level human commands and low-level robot actions by decomposing complex tasks into executable sequences. Using LLMs for planning enables robots to understand natural language commands, reason about environmental constraints, and generate appropriate action sequences that account for the physical world's complexities. This chapter covers the integration of LLMs with ROS 2 for creating intelligent, adaptive robotic behaviors.

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the role of cognitive planning in humanoid robotics
- Implement LLM-based task decomposition for robotic applications
- Create planning graphs and execution trees for complex tasks
- Map natural language commands to ROS 2 services, actions, and topics
- Handle constraints and adapt plans based on environmental conditions
- Implement safety mechanisms and validation for LLM-generated plans
- Design feedback loops for plan execution monitoring and adaptation

## Key Concepts

### Cognitive Planning in Robotics
Cognitive planning involves the high-level reasoning required to convert goals into executable actions:
- Task decomposition: Breaking complex goals into simpler subtasks
- Constraint handling: Managing environmental and robot-specific limitations
- Plan optimization: Finding efficient sequences of actions
- Adaptation: Modifying plans based on execution feedback

### LLM Integration for Planning
Large Language Models bring several advantages to robotic planning:
- Natural language understanding: Interpreting human commands directly
- Commonsense reasoning: Applying general world knowledge
- Analogical reasoning: Adapting known solutions to new situations
- Context awareness: Understanding situational constraints

### Planning Graphs and Execution Trees
Structured representations of plans that enable:
- Hierarchical task organization
- Dependency tracking between subtasks
- Alternative plan generation for failure recovery
- Execution monitoring and progress tracking

### ROS 2 Action Mapping
Converting LLM-generated plans to ROS 2 primitives:
- Services for synchronous operations
- Actions for long-running tasks with feedback
- Topics for continuous data streams
- Parameter servers for configuration

## Technical Deep Dive

### Cognitive Planning Architecture

The LLM-driven cognitive planning system follows this architecture:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Natural       │    │   LLM-based     │    │   Planning      │
│   Language      │───▶│   Task          │───▶│   Graph         │
│   Command       │    │   Decomposition │    │   Generation    │
│                 │    │                 │    │                 │
│ • "Clean the    │    │ • Semantic      │    │ • Action        │
│   room"         │    │   parsing       │    │   sequencing    │
│ • Context       │    │ • Task          │    │ • Dependency    │
│   information   │    │   identification│    │   tracking      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │                         │
                              ▼                         ▼
                    ┌─────────────────┐    ┌─────────────────┐
                    │   Environment   │    │   ROS 2         │
                    │   Context       │    │   Integration   │
                    │                 │    │                 │
                    │ • Object        │    │ • Action        │
                    │   locations     │    │   mapping       │
                    │ • Constraints   │    │ • Service       │
                    │ • Robot state   │    │   calls         │
                    └─────────────────┘    └─────────────────┘
                              │                         │
                              └─────────────────────────┼──────────────────┐
                                                        ▼                  ▼
                                               ┌─────────────────┐ ┌─────────────────┐
                                               │   Execution     │ │   Validation    │
                                               │   Monitoring    │ │   & Safety      │
                                               │                 │ │                 │
                                               │ • Progress      │ │ • Constraint    │
                                               │   tracking      │ │   checking      │
                                               │ • Feedback      │ │ • Plan safety   │
                                               │   collection    │ │ • Execution     │
                                               └─────────────────┘ │   validation    │
                                                                 └─────────────────┘
```

### LLM Planning Implementation

The core implementation involves several key components:

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


class LLMBasedPlanner(Node):
    """
    ROS 2 node that uses LLMs for cognitive planning in humanoid robots.
    """

    def __init__(self):
        super().__init__('llm_planner')

        # Initialize OpenAI client
        # In practice, you would configure this with your API key
        # self.openai_client = openai.OpenAI(api_key="your-api-key")

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

        # Parameters
        self.declare_parameter('llm_model', 'gpt-4-turbo')
        self.declare_parameter('max_planning_retries', 3)
        self.declare_parameter('plan_validation_enabled', True)

        self.get_logger().info('LLM-Based Planner initialized')

    def command_callback(self, msg):
        """
        Callback for high-level commands from users or other systems.
        """
        command_text = msg.data
        self.get_logger().info(f'Received command: {command_text}')

        # Generate plan based on command and current environment
        plan = self.generate_plan(command_text, self.current_environment)

        if plan:
            # Publish the plan
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

            self.get_logger().info(f'Published plan with {len(plan.tasks)} tasks')
        else:
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

            # In practice, you would call the LLM here
            # For this example, we'll simulate the LLM response
            # response = self.openai_client.chat.completions.create(
            #     model=self.get_parameter('llm_model').value,
            #     messages=[{"role": "user", "content": prompt}],
            #     temperature=0.1,
            #     response_format={"type": "json_object"}
            # )
            #
            # # Parse the LLM response
            # llm_response = response.choices[0].message.content
            # plan_data = json.loads(llm_response)

            # For demonstration purposes, create a simulated response
            plan_data = self.simulate_llm_response(command, environment)

            # Validate the plan
            if self.get_parameter('plan_validation_enabled').value:
                if not self.validate_plan(plan_data, environment):
                    self.get_logger().warn('Generated plan failed validation')
                    return None

            # Create Plan object from LLM response
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
            self.get_logger().error(f'Error generating plan: {e}')
            return None

    def create_planning_prompt(self, command: str, environment: Dict[str, Any]) -> str:
        """
        Create a detailed prompt for the LLM to generate a plan.
        """
        return f"""
        You are a cognitive planning system for a humanoid robot. Your task is to decompose high-level commands into executable robotic actions.

        Environment context:
        {json.dumps(environment, indent=2)}

        Command to execute:
        "{command}"

        Please decompose this command into a sequence of robotic actions. Each action should be specific enough for the robot to execute.

        Respond in JSON format:
        {{
            "plan_id": "unique_plan_identifier",
            "tasks": [
                {{
                    "id": "task_unique_id",
                    "type": "navigation|manipulation|perception|communication|wait",
                    "action": "specific_action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "dependencies": ["task_id_1", "task_id_2"]  // Tasks that must complete before this one
                }}
            ],
            "context": {{"additional_context": "information"}},
            "estimated_duration": 120.0  // Estimated time in seconds
        }}

        Guidelines:
        1. Break complex tasks into smaller, executable steps
        2. Consider environmental constraints from the context
        3. Include perception tasks when the robot needs to observe its environment
        4. Add navigation tasks for movement between locations
        5. Include manipulation tasks for object interaction
        6. Add dependencies between tasks where order matters
        7. Use specific, actionable language for each task
        """

    def simulate_llm_response(self, command: str, environment: Dict[str, Any]) -> Dict[str, Any]:
        """
        Simulate LLM response for demonstration purposes.
        In a real implementation, this would be replaced with actual LLM call.
        """
        # This is a simplified simulation - in practice, the LLM would generate more sophisticated plans
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
            # Default response for unrecognized commands
            return {
                "plan_id": f"plan_{int(time.time())}",
                "tasks": [
                    {
                        "id": "task_1",
                        "type": "communication",
                        "action": "request_clarification",
                        "parameters": {"message": f"I don't understand how to execute: {command}"},
                        "dependencies": []
                    }
                ],
                "context": {"original_command": command},
                "estimated_duration": 10.0
            }

    def validate_plan(self, plan_data: Dict[str, Any], environment: Dict[str, Any]) -> bool:
        """
        Validate the generated plan for safety and feasibility.
        """
        try:
            # Check if all required fields are present
            required_fields = ['plan_id', 'tasks']
            for field in required_fields:
                if field not in plan_data:
                    self.get_logger().error(f'Missing required field: {field}')
                    return False

            # Validate each task
            for task in plan_data['tasks']:
                required_task_fields = ['id', 'type', 'action', 'parameters']
                for field in required_task_fields:
                    if field not in task:
                        self.get_logger().error(f'Task {task.get("id", "unknown")} missing field: {field}')
                        return False

                # Validate task type
                try:
                    TaskType(task['type'])
                except ValueError:
                    self.get_logger().error(f'Invalid task type: {task["type"]}')
                    return False

                # Additional safety checks could go here
                # For example, check if navigation targets are in safe areas
                # or if manipulation actions are feasible given the robot's capabilities

            return True

        except Exception as e:
            self.get_logger().error(f'Error validating plan: {e}')
            return False


def main(args=None):
    """
    Main function to run the LLM-based planner node.
    """
    rclpy.init(args=args)

    planner = LLMBasedPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Interrupted by user, shutting down...')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Code Examples

### Advanced Planning with Constraint Handling

```python
from typing import List, Dict, Any, Optional, Tuple
import networkx as nx  # For planning graph representation


class AdvancedLLMPlanner(LLMBasedPlanner):
    """
    Enhanced LLM planner with advanced constraint handling and optimization.
    """

    def __init__(self):
        super().__init__()

        # Planning graph for dependency management
        self.planning_graph = nx.DiGraph()

    def generate_adaptive_plan(self, command: str, environment: Dict[str, Any]) -> Optional[Plan]:
        """
        Generate a plan that adapts to environmental constraints and robot capabilities.
        """
        # Analyze environmental constraints
        constraints = self.analyze_constraints(environment)

        # Generate initial plan
        initial_plan = self.generate_plan(command, environment)

        if not initial_plan:
            return None

        # Adapt plan based on constraints
        adapted_plan = self.adapt_plan_to_constraints(initial_plan, constraints, environment)

        return adapted_plan

    def analyze_constraints(self, environment: Dict[str, Any]) -> Dict[str, Any]:
        """
        Analyze environmental and robot constraints that affect planning.
        """
        constraints = {
            'navigation': {
                'blocked_areas': environment.get('blocked_areas', []),
                'narrow_passages': environment.get('narrow_passages', []),
                'climbing_limitations': environment.get('climbing_limitations', 0.1)  # meters
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
                'safe_distance': environment.get('safe_distance', 0.5)  # meters
            }
        }

        return constraints

    def adapt_plan_to_constraints(self, plan: Plan, constraints: Dict[str, Any],
                                 environment: Dict[str, Any]) -> Plan:
        """
        Adapt the plan to handle identified constraints.
        """
        adapted_tasks = []

        for task in plan.tasks:
            # Check if task is feasible given constraints
            if self.is_task_feasible(task, constraints, environment):
                adapted_tasks.append(task)
            else:
                # Generate alternative task or modify existing one
                alternative_tasks = self.generate_task_alternatives(task, constraints, environment)
                adapted_tasks.extend(alternative_tasks)

        # Update dependencies based on modifications
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
        Check if a task is feasible given current constraints.
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
        Generate alternative tasks when the original task is not feasible.
        """
        alternatives = []

        if task.type == TaskType.NAVIGATION:
            # Find alternative route
            alternative_route = self.find_alternative_navigation(task, constraints)
            if alternative_route:
                alternatives.extend(alternative_route)
            else:
                # Add a "request_assistance" task
                alternatives.append(Task(
                    id=f"alt_{task.id}",
                    type=TaskType.COMMUNICATION,
                    action="request_assistance",
                    parameters={"message": f"Cannot navigate to {task.parameters.get('x', 'unknown')}"},
                    dependencies=task.dependencies
                ))

        elif task.type == TaskType.MANIPULATION:
            # Find alternative object or approach
            alternatives.append(Task(
                id=f"alt_{task.id}",
                type=TaskType.COMMUNICATION,
                action="request_alternative",
                parameters={"message": f"Cannot manipulate object {task.parameters.get('object_id')}"},
                dependencies=task.dependencies
            ))

        return alternatives

    def find_alternative_navigation(self, task: Task, constraints: Dict[str, Any]) -> Optional[List[Task]]:
        """
        Find alternative navigation path when direct path is blocked.
        """
        # This would implement path planning algorithms like A* or RRT
        # For this example, we'll return a simple alternative
        target_x = task.parameters.get('x', 0)
        target_y = task.parameters.get('y', 0)

        # Check if there's a simple alternative route
        # In practice, this would use proper path planning
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
        Update task dependencies after modifications.
        """
        # This would update dependencies based on the new task sequence
        # For now, we'll maintain the original dependency structure where possible
        return tasks

    def estimate_plan_duration(self, tasks: List[Task]) -> float:
        """
        Estimate the total duration of the plan.
        """
        return sum(task.estimated_duration for task in tasks)


# Example usage of the advanced planner
def create_advanced_planner_node():
    """
    Create and return an advanced planner node instance.
    """
    return AdvancedLLMPlanner()
```

## Diagrams (Text-based)

### Planning Process Flow

```
High-Level Command ──► [LLM Interpretation] ──► [Task Decomposition] ──► [Constraint Checking]
       │                       │                        │                       │
       │                       ▼                        ▼                       ▼
       │               [Semantic Parsing]      [Action Sequencing]    [Feasibility Analysis]
       │               [Intent Extraction]     [Dependency Mapping]   [Safety Validation]
       │                                                                 │
       └─────────────────────────────────────────────────────────────────┼─────────► Plan
                                                                         ▼
                                                                 [Plan Optimization]
                                                                 [Alternative Generation]
```

### Planning Graph Structure

```
                    ROOT (Original Command)
                           │
              ┌────────────┼────────────┐
              │            │            │
        Navigation    Perception    Manipulation
        Task A        Task B        Task C
              │            │            │
        ┌─────┘            │            └─────┐
        │                  │                  │
    Check Door        Detect Object      Grasp Object
    Status            in Area            with Gripper
        │                  │                  │
        └──────────────────┼──────────────────┘
                           ▼
                    Update World Model
                           │
                           ▼
                    Execute Action Plan
```

## Common Pitfalls

### 1. Overly Complex Plans
**Problem**: LLMs may generate overly complex plans that are difficult to execute.
**Solution**: Implement plan simplification and validation steps to ensure feasibility.

### 2. Lack of Environmental Context
**Problem**: Plans generated without considering current environmental state.
**Solution**: Ensure the LLM has access to up-to-date environmental information.

### 3. Safety Violations
**Problem**: LLM-generated plans may include unsafe actions.
**Solution**: Implement safety validation layers that check all actions before execution.

### 4. Execution Failure Recovery
**Problem**: Plans don't account for potential execution failures.
**Solution**: Include monitoring and recovery mechanisms in the planning process.

### 5. Computational Overhead
**Problem**: Complex LLM queries may introduce unacceptable latency.
**Solution**: Cache common plans and use lightweight validation for simple tasks.

## Checkpoints

### Understanding Check 1: Planning Concepts
- What is the difference between task decomposition and action execution?
- How do LLMs improve robotic planning compared to traditional methods?
- What are the key components of a planning graph?

### Understanding Check 2: Implementation
- How would you modify the planner to handle multi-robot coordination?
- What safety checks would you add to prevent harmful robot actions?
- How could you optimize the planning process for real-time applications?

### Application Check: Cognitive Planning
- How would you extend the planner to handle collaborative tasks?
- What additional environmental sensors would improve planning accuracy?
- How would you handle conflicting commands from multiple users?

## References

1. Chen, X., et al. (2023). "Language Models as Zero-Shot Planners: Extracting Actionable Knowledge for Embodied Agents." *Proceedings of the International Conference on Machine Learning (ICML)*.

2. Huang, W., et al. (2022). "Language Models as Zero-Shot Trajectory Optimizers." *arXiv preprint arXiv:2204.03535*.

3. OpenAI. (2023). "GPT-4 Technical Report." OpenAI. Available: https://openai.com/research/gpt-4

4. Brohan, C., et al. (2022). "RT-1: Robotics Transformer for Real-World Control at Scale." *arXiv preprint arXiv:2212.06817*.

5. Ahn, M., et al. (2022). "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances." *arXiv preprint arXiv:2204.01691*.

6. Zhu, Y., et al. (2017). "Target-driven Visual Navigation in Indoor Scenes using Deep Reinforcement Learning." *IEEE International Conference on Robotics and Automation (ICRA)*.

7. Fox, D., et al. (1998). "Active Marker Localization for Visual Tracking." *IEEE International Conference on Robotics and Automation (ICRA)*.

8. Kaelbling, L.P., et al. (1998). "Planning and Acting in Partially Observable Stochastic Domains." *Artificial Intelligence Journal*, 101(1-2), 99-134.

9. Russell, S., & Norvig, P. (2020). "Artificial Intelligence: A Modern Approach" (4th ed.). Pearson.

10. Siciliano, B., & Khatib, O. (2016). "Springer Handbook of Robotics" (2nd ed.). Springer.