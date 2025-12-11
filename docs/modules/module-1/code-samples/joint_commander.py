import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import JointPositionCommand # Assuming custom message

class JointCommander(Node):

    def __init__(self):
        super().__init__('joint_commander')
        self.publisher_ = self.create_publisher(JointPositionCommand, '/joint_commands', 10)
        self.declare_parameter('joint_name', 'shoulder_joint') # Default joint name
        self.declare_parameter('target_position', 0.0)      # Default target position
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Joint Commander Node started.')

    def timer_callback(self):
        joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
        target_position = self.get_parameter('target_position').get_parameter_value().double_value

        msg = JointPositionCommand()
        msg.joint_name = joint_name
        msg.position = target_position
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing command: Joint="{msg.joint_name}", Position={msg.position}')

def main(args=None):
    rclpy.init(args=args)
    joint_commander = JointCommander()
    rclpy.spin(joint_commander)
    joint_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
