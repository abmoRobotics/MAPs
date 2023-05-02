#!/usr/bin/env python3
# utils.py
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
import yaml
import numpy as np
import os
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import os

from ros2param.api import call_list_parameters, call_get_parameters, get_value
from ros2node.api import get_absolute_node_name
from rclpy.parameter import PARAMETER_SEPARATOR_STRING
from ros2node.api import get_node_names

def create_publisher_array(node: Node, n_robots: int, topic_prefix: str, topic_name: str, msg_type: type):
    """
    Create a list of ROS publishers for a given number of robots.

    :param node: The ROS Node object that will be used to create the publishers.
    :type node: Node

    :param n_robots: The number of publishers to create.
    :type n_robots: int

    :returns: A list of ROS publishers created by the given node for each robot, with the topic name and index based on the robot number.
    :rtype: List[Publisher]
    """
    publishers = []
    for i in range(n_robots):
        node.get_logger().info('created topic: ' + topic_prefix + str(i) + topic_name)
        node.get_logger().info('n_robots: ' + str(n_robots))
        publishers.append(node.create_publisher(msg_type, topic_prefix+str(i)+topic_name, 10))
    return publishers

def destroy_publisher_array(node: Node, publishers: int):
    """s
    Create a list of ROS publishers for a given number of robots.

    :param node: The ROS Node object that will be used to create the publishers.
    :type node: Node

    :param n_robots: The number of publishers to create.
    :type n_robots: int

    :returns: A list of ROS publishers created by the given node for each robot, with the topic name and index based on the robot number.
    :rtype: List[Publisher]
    """
    for i in range(len(publishers)):
        node.get_logger().info('Destroy all topics')
        node.destroy_publisher(publishers[i])
    return publishers

def create_joint_state_message_array(joint_names: str, n_msgs: int) -> JointState:
    """
    Creates an array of JointState messages with the given number of messages and joint names.

    Args:
        joint_names (str): A string representing the names of the joints in the messages.
        n_msgs (int): The number of JointState messages to create.

    Returns:
        List[JointState]: An array of JointState messages, each with the same joint names.
    """
    msgs = []
    msg = JointState()
    for i in range(n_msgs):
        zero_array = [0.0] * len(joint_names)
        msg.name = joint_names
        msg.position = zero_array
        msg.velocity = zero_array
        msg.effort = zero_array
        msgs.append(msg)
    return msgs

def load_yaml_file(node_name: str) -> dict:
    config_path = os.path.join(
        get_package_share_directory('orchestrator'),
        'config',
        'config.yaml')
    
    with open(config_path, 'r') as f:
        config = yaml.full_load(f)
        config = config[node_name]["ros__parameters"]
        return config   


def yaml_position_generate(node: Node, number_of_robots: int):
    positions = {}
    with open('src/orchestrator/config/initial_position.yaml', 'w') as yaml_file:
        for i in range(number_of_robots):
            positions['shuttle'+str(i+1)+'_position'] = []
        node.get_logger().info(str(positions))
        node_n = {str(node.get_name()): {'ros__parameters': positions}} 

        yaml_out = yaml.dump(node_n, yaml_file, sort_keys=False) 
        node.get_logger().info(str(yaml_out))
 

def insert_dict(dictionary, key, value):
    split = key.split(".", 1)
    if len(split) > 1:
        if not split[0] in dictionary:
            dictionary[split[0]] = {}
        insert_dict(dictionary[split[0]], split[1], value)
    else:
        dictionary[key] = value

def get_parameter_values(node, node_name, params):
    response = call_get_parameters(
        node=node, node_name=node_name,
        parameter_names=params)

    # requested parameter not set
    if not response.values:
        return '# Parameter not set'

    # extract type specific value
    return [get_value(parameter_value=i) for i in response.values]       

#TODO FIX
def save_yaml(self):
    node_names = get_node_names(node=self, include_hidden_nodes=False)

    yaml_output = {}

    for node in node_names:
        print(node.full_name)
        print("jeg sidder fast2")
        response = call_list_parameters(node=self, node_name=node.full_name)
        print("jeg sidder fast3")
        response = sorted(response)
        print("jeg sidder fast4")
        parameter_values = get_parameter_values(self, node.full_name, response)
        print("jeg sidder fast5")
        #yaml_output = {node.name: {'ros__parameters': {}}}
        yaml_output[node.name] = {'ros__parameters': {}}
        for param_name, pval in zip(response, parameter_values):
            print("jeg sidder fast")
            insert_dict(
                yaml_output[node.name]['ros__parameters'], param_name, pval)
        print(yaml_output)
    print("NU2")
    print(yaml_output)
    with open(os.path.join("", "tester3" + '.yaml'), 'w') as yaml_file:
        yaml.dump(yaml_output, yaml_file, default_flow_style=False)


def create_action_server_array(node: Node, action_type: type, topic_prefix: str, callback_type: type, n_robots: int):
    """
    Create a list of ROS action servers for a given number of robots.

    :param node: The ROS Node object that will be used to create the action.
    :type node: Node

    :param action_type: The type of action that is asigned for the robot 
    :type action_type: type

    :param n_robots: The number of servers to create.
    :type n_robots: int

    :returns: A list of ROS publishers created by the given node for each robot, with the topic name and index based on the robot number.
    :rtype: List[Publisher]
    """
    action_servers = []
    for i in range(n_robots):
        node.get_logger().info('created action server: ' + topic_prefix + str(i))
        node.get_logger().info('n_robots: ' + str(topic_prefix))
        action_servers.append(ActionServer(node, action_type, topic_prefix + str(i), callback_type))
    return action_servers

def create_action_client_array(node: Node, action_type: type, topic_prefix: str, n_robots: int):
    """
    Create a list of ROS action clients for a given number of robots.

    :param node: The ROS Node object that will be used to create the action client.
    :type node: Node

    :param action_type: The type of action that is asigned for the robot 
    :type action_type: type

    :param n_robots: The number of clients to create.
    :type n_robots: int

    :returns: A list of ROS action clients created by the given node for each robot, with the topic prefic and index based on the robot number.
    :rtype: List[Action Clients]
    """
    action_clients = []
    for i in range(n_robots):
        node.get_logger().info('created action: ' + topic_prefix + str(i))
        node.get_logger().info('n_robots: ' + str(topic_prefix))
        action_clients.append(ActionClient(node, action_type, topic_prefix + str(i)))
    return action_clients