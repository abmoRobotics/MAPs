import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
from spatialmath import SE3
import roboticstoolbox as rtb

from utils import create_joint_state_message_array, create_publisher_array, load_yaml_file
from utils import KR4R600, KR3R540


class Manipulator(Node):

    def __init__(self):
        super().__init__('manipulator')
        # Define publishers and messages
        manipulator_config = load_yaml_file(self.get_name())
        print(manipulator_config)
        for key, value in manipulator_config.items():
            if "position" in key:
                self.declare_parameter(key)
                param_value = rclpy.Parameter(
                    key,
                    rclpy.Parameter.Type.DOUBLE_ARRAY,
                    value,
                )
                self.set_parameters([param_value])

        self.kuka540 = KR3R540()
        self.kuka600 = KR4R600()
        self.msg = JointState()

        self.number_of_manipulators = 1
    
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.publisher_array = create_publisher_array(self, 
                                                      self.number_of_manipulators,
                                                      topic_prefix=self.get_name(), 
                                                      topic_name='/joint_command',
                                                      msg_type=JointState)
        self.msg_array = create_joint_state_message_array(joint_names, 
                                                          self.number_of_manipulators)

        # Define callback timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        ## Test values
        pose_test1 = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
        pose_test2 = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])

        #sol1 = self.kuka540.ik_lm_chan(pose_test1)
        sol2 = self.kuka600.ik_lm_chan(pose_test2)
        sol3 = self.kuka600.ik_lm_chan(pose_test1)

        self.qt1 = rtb.jtraj(self.kuka540.home, sol3[0], 50)
        self.qt2 = rtb.jtraj(self.kuka600.home, sol2[0], 50)

        self.pos2 = self.qt2.q.tolist()

    def timer_callback(self):


        for idx, publisher in enumerate(self.publisher_array):
            for time, pos in enumerate(self.pos2):
                self.msg.position = pos
                publisher.publish(self.msg)


def main(args=None):
    """Initializes the Manipulator node and spins it until it is destroyed."""
    rclpy.init(args=args)

    manipulator_node = Manipulator()

    rclpy.spin(manipulator_node)

    manipulator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
