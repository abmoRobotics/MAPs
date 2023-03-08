#!/usr/bin/env python3
# utils.py
from rclpy.node import Node
import yaml
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

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



def yaml_position_generate(node: Node, number_of_robots: int):
    positions = {}
    with open('src/orchestrator/config/initial_position.yaml', 'w') as yaml_file:
        for i in range(number_of_robots):
            positions['shuttle'+str(i+1)+'_position'] = []
        node.get_logger().info(str(positions))
        node_n = {str(node.get_name()): {'ros__parameters': positions}} 

        yaml_out = yaml.dump(node_n, yaml_file, sort_keys=False) 
        node.get_logger().info(str(yaml_out))
 
        