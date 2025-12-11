# ai_commander_node.py
import rclpy
from rclpy.node import Node
from humanoid_nervous_system.msg import JointCommand # Your custom message

class AICommander(Node):
    def __init__(self):
        super().__init__('ai_commander')
        self.publisher_ = self.create_publisher(JointCommand, '/joint_command', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Publish every second
        self.joint_names = ["right_shoulder", "left_shoulder"]
        self.target_positions = [0.5, -0.5]
        self.current_joint_idx = 0
        self.get_logger().info('AI Commander Node started.')

    def timer_callback(self):
        joint_name = self.joint_names[self.current_joint_idx]
        target_position = self.target_positions[self.current_joint_idx]

        msg = JointCommand()
        msg.joint_name = joint_name
        msg.position = target_position
        self.publisher_.publish(msg)
        self.get_logger().info(f'Commanding {joint_name} to {target_position}')

        self.current_joint_idx = (self.current_joint_idx + 1) % len(self.joint_names)

def main(args=None):
    rclpy.init(args=args)
    ai_commander = AICommander()
    rclpy.spin(ai_commander)
    ai_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
