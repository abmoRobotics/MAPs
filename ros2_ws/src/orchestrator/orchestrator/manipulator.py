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
from utils import ValueIDManager
from typing import List
from utils import (create_joint_state_message_array, 
                    create_publisher_array, 
                    load_yaml_file,
                    create_action_server_array,
                    destroy_publisher_array)

from utils import KR4R600, KR3R540, ClientAsync
from geometry_msgs.msg import Pose

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
        self.number_of_manipulators = 0
        self.manipulator_positions = []
        self.value_id_manager = ValueIDManager()

        self.kuka540 = KR3R540()
        self.kuka600 = KR4R600()

    
        name600 = self.number_of_manipulators * [self.kuka600._name]

        self.name_of_manipulators = name600
        self.publisher_array = []
        self.msg = []
        self.action_server: List[ActionServer] = []

        self.manipulators = {'name': 'manipulator', 'position': [0, 0, 0] }
    
        self.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
     
        self.publisher_array = create_publisher_array(self, 
                                                            self.number_of_manipulators,
                                                            topic_prefix=self.kuka600._name, 
                                                            topic_name='/joint_command',
                                                            msg_type=JointState)
    
        self.msg = create_joint_state_message_array(self.joint_names, 
                                                            self.number_of_manipulators)
    
        self.action_server = create_action_server_array(self, 
                                                Man, 
                                                topic_prefix=self.kuka600._name, 
                                                callback_type=self.action_mani_callback, 
                                                n_robots=self.number_of_manipulators)
        

        # Define callback timer
        timer_period = 0.1  # seconds
        self.new_goal = True
        self.pos = []
        self.vel = []
        self.timer = self.create_timer(timer_period, self.mani_callback)

        self.service_client = ClientAsync()

    def mani_callback(self):
        
        request = GetParameters.Request()
        request.names = ['num_of_manipulators', 'robot_position']
        future = self.client.call_async(request=request)
        future.add_done_callback(self.callback_global_param)
        removed_array_index, removed_ids = self.value_id_manager.sync_array(self.manipulator_positions)
        
        if self.number_of_manipulators != len(self.publisher_array):
            print(f'Number of manipulators: {self.number_of_manipulators}')
            print(f'Number of publishers: {len(self.publisher_array)}')
            print(f'Remove ids: {removed_array_index}')
        if self.number_of_manipulators < len(self.publisher_array):
            self.get_logger().error("Number of manipulators is:" + str(self.number_of_manipulators))
            
            removed_array_index.sort(reverse=True)
            self.get_logger().error("Remove ids is:" + str(removed_array_index))
            for array_idx, idx in zip(removed_array_index, removed_ids):
                self.destroy_publisher(self.publisher_array[array_idx])
                self.action_server[array_idx].destroy()
                self.service_client.remove_robot(self.kuka600._name, idx)
                self.publisher_array.pop(array_idx)
                self.msg.pop(array_idx)
                self.action_server.pop(array_idx)
                
        
        if self.number_of_manipulators > len(self.publisher_array):
            # Add new manipulator'
            print(self.value_id_manager.id_to_value)
            if len(list(self.value_id_manager.id_to_value.keys())) == 0:
                manipulator_id = 0
            else:
                print(f'ID to value: {self.value_id_manager.id_to_value.keys()}')
                manipulator_id = list(self.value_id_manager.id_to_value.keys())[-1]
            manipulator_position = self.value_id_manager.id_to_value[manipulator_id]
            pose = self.transform_to_pose(manipulator_position)
            
            self.publisher_array.append(self.create_publisher(JointState, self.kuka600._name + str(manipulator_id) + '/joint_command', 10))
            self.action_server.append(ActionServer(self, Man, self.kuka600._name + str(manipulator_id), self.action_mani_callback))
            self.msg.append(JointState())
            # pose = Pose()
            # import random
            # ros2 service call /segment/add tutorial_interfaces/srv/PoseSrv "{name: 'segment_00', pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
            # pose.position.x = float(random.random())
            # pose.position.y = float(random.random())
            # pose.position.z = float(random.random())
            # pose.orientation.x = float(0)
            # pose.orientation.y = float(0)
            # pose.orientation.z = float(0)
            # pose.orientation.w = float(1)
            print(self.kuka600._name + f'{manipulator_id:02}')
            self.service_client.spawn_robot(self.kuka600._name, manipulator_id, pose)
            #destroy_publisher_array(self, self.publisher_array)
            # self.msg = []
            # self.action_server = []
            
            # self.publisher_array = create_publisher_array(self, 
            #                                                     self.number_of_manipulators,
            #                                                     topic_prefix=self.kuka600._name, 
            #                                                     topic_name='/joint_command',
            #                                                     msg_type=JointState)
        
            # self.msg = create_joint_state_message_array(self.joint_names, 
            #                                                     self.number_of_manipulators)
        
            # self.action_server = create_action_server_array(self, 
            #                                         Man, 
            #                                         topic_prefix=self.kuka600._name, 
            #                                         callback_type=self.action_mani_callback, 
            #                                         n_robots=self.number_of_manipulators)
    
        self.new_goal = False
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
            self.number_of_manipulators = result.values[0].integer_value
            self.manipulator_positions = result.values[1].double_array_value
            self.number_of_manipulators = len(self.manipulator_positions)
        except Exception as e:
            self.get_logger().warn("Service call failed inside manipulator %r" % (e,))


    def transform_to_pose(self, pos):
        pos = str(pos)
        y, x = pos.split('.')
        pose = Pose()
        pose.position.x = float(x)*0.18+0.12
        pose.position.y = float(y)*0.18+0.12
        pose.position.z = float(0.9)
        pose.orientation.x = float(0)
        pose.orientation.y = float(0)
        pose.orientation.z = float(0)
        pose.orientation.w = float(1)
        return pose
    
def main(args=None):
    """Initializes the Manipulator node and spins it until it is destroyed."""
    rclpy.init(args=args)

    manipulator_node = Manipulator()

    rclpy.spin(manipulator_node)

    manipulator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
