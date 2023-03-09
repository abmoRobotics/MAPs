import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from utils import create_joint_state_message_array, create_publisher_array, save_yaml, insert_dict, get_parameter_values
from ros2param.api import call_list_parameters, call_get_parameters, get_value
from ros2node.api import get_absolute_node_name
from rclpy.parameter import PARAMETER_SEPARATOR_STRING
import os 
import yaml
from ros2cli.node.strategy import NodeStrategy
from ros2cli.node.direct import DirectNode
from ros2node.api import get_node_names

class GUI(Node):

    def __init__(self):
        super().__init__('gui')


        
        self.msg = Pose()



        # Define callback timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        joint_names = ['x_translation', 'y_translation', 'z_translation', 'x_rotation', 'y_rotation', 'z_rotation']
        publisher_array= create_publisher_array(self, number_of_shuttles, topic_prefix="configuration/shuttle", topic_name='/initialPosition', msg_type=Pose)
        self.msg_array = create_joint_state_message_array(joint_names, number_of_shuttles)

        try:
            for idx, publisher in enumerate(publisher_array):
                self.msg.position.x = param[idx][0]
                self.msg.position.y = param[idx][1]
                self.msg.position.z = param[idx][2]
                publisher.publish(self.msg)
        except:
            pass

        #self.save_yaml()

    def timer_callback(self):
        try:
            pass
            #self.get_logger().info(str(self.get_parameter('shuttle1_position').get_parameter_value().double_array_value))
        except:
            pass
    
    def save_yaml(self):
        node_names = get_node_names(node=self, include_hidden_nodes=False)

        yaml_output = {}

        for node in node_names:
            print(node.full_name)
            response = call_list_parameters(node=self, node_name=node.full_name)
            response = sorted(response)
            parameter_values = self.get_parameter_values(self, node.full_name, response)
            #yaml_output = {node.name: {'ros__parameters': {}}}
            yaml_output[node.name] = {'ros__parameters': {}}
            for param_name, pval in zip(response, parameter_values):
                self.insert_dict(
                    yaml_output[node.name]['ros__parameters'], param_name, pval)
        
        with open(os.path.join("", "tester3" + '.yaml'), 'w') as yaml_file:
            yaml.dump(yaml_output, yaml_file, default_flow_style=False)

    def insert_dict(self, dictionary, key, value):
        split = key.split(".", 1)
        if len(split) > 1:
            if not split[0] in dictionary:
                dictionary[split[0]] = {}
            self.insert_dict(dictionary[split[0]], split[1], value)
        else:
            dictionary[key] = value

    @staticmethod
    def get_parameter_values(node, node_name, params):
        response = call_get_parameters(
            node=node, node_name=node_name,
            parameter_names=params)

        # requested parameter not set
        if not response.values:
            return '# Parameter not set'

        # extract type specific value
        return [get_value(parameter_value=i) for i in response.values]


def main(args=None):
    """Initializes the gui node and spins it until it is destroyed."""
    rclpy.init(args=args)

    gui_node = GUI()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
