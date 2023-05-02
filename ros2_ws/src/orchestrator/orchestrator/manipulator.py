import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from robot_actions.action import Man

from sensor_msgs.msg import JointState
import numpy as np
import time
from spatialmath import SE3
import roboticstoolbox as rtb
import time

from utils import (create_joint_state_message_array, 
                    create_publisher_array, 
                    load_yaml_file,
                    create_action_server_array)

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

        number_of_kuka540 = 2
        number_of_kuka600 = 2

        name540 = number_of_kuka540 * [self.kuka540._name]
        name600 = number_of_kuka600 * [self.kuka600._name]

        self.name_of_manipulators = np.concatenate((name600, name540))
        self.publisher_array = []
        self.msg = []
        self.action_server = []

        self.manipulators = {'name': 'manipulator', 'position': [0, 0, 0] }
    
        joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        for idx, name in enumerate(self.name_of_manipulators):
            self.publisher_array.append(create_publisher_array(self, 
                                                                1,
                                                                topic_prefix=name, 
                                                                topic_name='/joint_command',
                                                                msg_type=JointState))
        
            self.msg.append(create_joint_state_message_array(joint_names, 
                                                                1))
        
            self.action_server.append(create_action_server_array(self, 
                                                    Man, 
                                                    topic_prefix=name, 
                                                    callback_type=self.action_mani_callback, 
                                                    n_robots=1))
            
        print(self.action_server)

        # Define callback timer
        timer_period = 0.1  # seconds
        self.new_goal = True
        self.pos = []
        self.vel = []
        self.timer = self.create_timer(timer_period, self.mani_callback)

    def mani_callback(self):

        if self.new_goal:
            for idx, publisher in enumerate(self.publisher_array):
                    #print(len(self.pos))
                    pose = SE3.Trans(0.5, 0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
                    sol = self.kuka600.ik_lm_chan(pose)
                    self.qt = rtb.jtraj(self.kuka600.home, self.kuka600.qz, 100)
                    #print(str(idx)+"  mellemrum   "+str(publisher))
                    self.pos.append(self.qt.q.tolist())
                    self.vel.append(self.qt.qd.tolist())
                    

        if len(self.pos) > 7 and len(self.vel) > 7 and self.new_goal:
            for idy, pos in enumerate(self.pos[0]):
                for idx, publisher in enumerate(self.publisher_array):
                    #print("SELF:POS: " + str(self.pos))
                    time.sleep(0.01) # Need to be removed
                    self.msg[idx][0].position = self.pos[idx][idy]
                    self.msg[idx][0].velocity = self.vel[idx][idy]
                    #print("INDI POS: " + str(self.pos[idy][idx]))
                    publisher[0].publish(self.msg[idx][0])

            self.new_goal = False
            print("New goal is: " + str(self.new_goal))
        
            
            # for idx, publisher in enumerate(self.publisher_array):
            #     print("HEKKK" + str(self.pos[idx][-1]))
            #     self.pos[idx] = self.pos[idx][-1]
            #     self.vel[idx] = self.vel[idx][-1]
            #     #print(self.pos)
            #     #print(self.vel)

        
    def action_mani_callback(self):


        msg = Manipulator.goal_feedback()
        pass


def main(args=None):
    """Initializes the Manipulator node and spins it until it is destroyed."""
    rclpy.init(args=args)

    manipulator_node = Manipulator()

    rclpy.spin(manipulator_node)

    manipulator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
