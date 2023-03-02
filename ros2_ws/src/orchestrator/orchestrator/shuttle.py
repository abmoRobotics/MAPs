import rclpy
from rclpy.node import Node

from utils.utils import create_joint_state_message_array, create_publisher_array


class Manipulator(Node):

    def __init__(self):
        super().__init__('shuttle')
        # Define publishers and messages
        number_of_shuttles = 10
        joint_names = ['x_translation', 'y_translation', 'z_translation', 'x_rotation', 'y_rotation', 'z_rotation']
        self.publisher_array = create_publisher_array(self, number_of_shuttles, '/joint_command')
        self.msg_array = create_joint_state_message_array(joint_names, number_of_shuttles)

        # Define callback timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        for idx, publisher in enumerate(self.publisher_array):
            publisher.publish(self.msg_array[idx])

def main(args=None):
    rclpy.init(args=args)

    manipulator_node = Manipulator()

    rclpy.spin(manipulator_node)

    manipulator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
