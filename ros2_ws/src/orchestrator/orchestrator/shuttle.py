import rclpy
from rclpy.node import Node

import utils
from utils import ShuttleMode

class Shuttle(Node):

    def __init__(self):
        super().__init__('shuttle')
        # Define publishers and messages
        self.declare_parameter('num_of_shuttles')
        self.declare_parameter('sim_shuttle')
        number_of_shuttles = self.get_parameter('num_of_shuttles').get_parameter_value().integer_value
        sim_shuttle = self.get_parameter('sim_shuttle').get_parameter_value().bool_value

        if sim_shuttle == False:
            number_of_shuttles = 0
        else:
            joint_names = ['x_translation', 'y_translation', 'z_translation', 'x_rotation', 'y_rotation', 'z_rotation']
            self.publisher_array = utils.create_publisher_array(self, number_of_shuttles, '/joint_command')
            self.msg_array = utils.create_joint_state_message_array(joint_names, number_of_shuttles)

        # Define callback timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        for idx, publisher in enumerate(self.publisher_array):
            publisher.publish(self.msg_array[idx])

            

def main(args=None):
    """Initializes the Shuttle node and spins it until it is destroyed."""
    rclpy.init(args=args)

    shuttle_node = Shuttle()

    rclpy.spin(shuttle_node)

    shuttle_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
