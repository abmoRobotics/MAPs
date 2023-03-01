import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState
import threading
from utils.manipulator_utils import *


class Manipulator(Node):

    def __init__(self):
        super().__init__('manipulator')
        
        ## Define publishers and messages
        number_of_shuttles = 5
        joint_names = ['x_translation','y_translation','z_translation','x_rotation','y_rotation','z_rotation']
        self.publishers = create_publisher_array(self, number_of_shuttles)
        self.msgs = create_joint_state_message_array(joint_names, number_of_shuttles)


        ## Define callback timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Manipulator()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
