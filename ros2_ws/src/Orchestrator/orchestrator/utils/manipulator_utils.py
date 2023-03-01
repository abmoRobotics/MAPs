
from rclpy.node import Node
from sensor_msgs.msg import JointState



def create_publisher_array(node: Node, n_robots: int):
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
        node.get_logger().info('Publishing' + str(i))
        publishers.append(node.create_publisher(JointState, '/shuttle'+str(i)+'/joint_command', 10))
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
        msg.name = joint_names
        msgs.append(msg)
    return msgs
    

    #        msg.position = [1.0,2.0,3.0,4.0,5.0,6.0]