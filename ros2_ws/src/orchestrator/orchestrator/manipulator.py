import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer
from robot_actions.action import Man

from sensor_msgs.msg import JointState
import numpy as np
import time
from rcl_interfaces.srv import GetParameters
from spatialmath import SE3
import roboticstoolbox as rtb
import time

from utils import (create_joint_state_message_array, 
                    create_publisher_array, 
                    load_yaml_file,
                    create_action_server_array,
                    destroy_publisher_array)

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

        self.client = self.create_client(GetParameters,
                                        '/global_parameter_server/get_parameters')
        
        # Request parameter from the /gui node
        self.num_mani = 0


        self.kuka540 = KR3R540()
        self.kuka600 = KR4R600()

    
        name600 = self.num_mani * [self.kuka600._name]

        self.name_of_manipulators = name600
        self.publisher_array = []
        self.msg = []
        self.action_server = []

        self.manipulators = {'name': 'manipulator', 'position': [0, 0, 0] }
    
        self.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
     
        self.publisher_array = create_publisher_array(self, 
                                                            self.num_mani,
                                                            topic_prefix=self.kuka600._name, 
                                                            topic_name='/joint_command',
                                                            msg_type=JointState)
    
        self.msg = create_joint_state_message_array(self.joint_names, 
                                                            self.num_mani)
    
        self.action_server = create_action_server_array(self, 
                                                Man, 
                                                topic_prefix=self.kuka600._name, 
                                                callback_type=self.action_mani_callback, 
                                                n_robots=self.num_mani)
        

        # Define callback timer
        timer_period = 0.1  # seconds
        self.new_goal = True
        self.pos = []
        self.vel = []
        self.timer = self.create_timer(timer_period, self.mani_callback)

    def mani_callback(self):
        
        request = GetParameters.Request()
        request.names = ['num_of_manipulators']
        future = self.client.call_async(request=request)
        future.add_done_callback(self.callback_global_param)

        if self.num_mani > len(self.publisher_array) or self.num_mani < len(self.publisher_array):
            self.get_logger().error("Number of manipulators is:" + str(self.num_mani))
            destroy_publisher_array(self, self.publisher_array)
            self.msg = []
            self.action_server = []
            
            self.publisher_array = create_publisher_array(self, 
                                                                self.num_mani,
                                                                topic_prefix=self.kuka600._name, 
                                                                topic_name='/joint_command',
                                                                msg_type=JointState)
        
            self.msg = create_joint_state_message_array(self.joint_names, 
                                                                self.num_mani)
        
            self.action_server = create_action_server_array(self, 
                                                    Man, 
                                                    topic_prefix=self.kuka600._name, 
                                                    callback_type=self.action_mani_callback, 
                                                    n_robots=self.num_mani)
    

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
                    self.msg[idx].position = self.pos[idx][idy]
                    self.msg[idx].velocity = self.vel[idx][idy]
                    #print("INDI POS: " + str(self.pos[idy][idx]))
                    publisher.publish(self.msg[idx])

            self.new_goal = False
            print("New goal is: " + str(self.new_goal))


        
    def action_mani_callback(self, goal_handle):


        f_msg = Man.Feedback()
        f_msg.goalfeedback = "Running"

        
        self.get_logger().info("Goal request" + str(goal_handle.request.goal))

        self.get_logger().info('Feedback: {0}'.format(f_msg.goalfeedback))
        goal_handle.publish_feedback(f_msg)
        time.sleep(1)

        goal_handle.succeed()

        result = Man.Result()
        result.goalresult = " Done running"

        return result
    
    def callback_global_param(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn("Service call failed inside manipulator %r" % (e,))
        else:
            self.param = result.values[0]
            self.num_mani = self.param.integer_value
            # self.get_logger().info("Number of manipulators is: %s" % (self.num_mani,))


def main(args=None):
    """Initializes the Manipulator node and spins it until it is destroyed."""
    rclpy.init(args=args)

    manipulator_node = Manipulator()

    rclpy.spin(manipulator_node)

    manipulator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
