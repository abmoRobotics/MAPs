import rclpy
from rclpy.node import Node

from utils import create_joint_state_message_array, create_publisher_array


class Manipulator(Node):

    def __init__(self):
        super().__init__('manipulator')
        # Define publishers and messages
        number_of_shuttles = 10
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.publisher_array = create_publisher_array(self, number_of_shuttles, '/joint_command')
        self.msg_array = create_joint_state_message_array(joint_names, number_of_shuttles)

        # Define callback timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        for idx, publisher in enumerate(self.publisher_array):
            publisher.publish(self.msg_array)

def main(args=None):
    """Initializes the Manipulator node and spins it until it is destroyed."""
    rclpy.init(args=args)

    manipulator_node = Manipulator()

    rclpy.spin(manipulator_node)

    manipulator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
