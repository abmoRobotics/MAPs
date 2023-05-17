import rclpy
from rclpy.node import Node


from utils import create_joint_state_message_array, create_publisher_array, destroy_publisher_array, load_yaml_file, save_yaml
from utils import ShuttleMode
from sensor_msgs.msg import JointState
import random
import numpy as np

def generate_random_goals(num_shuttles):
    desired_positions = np.random.rand(num_shuttles, 2)
    desired_positions[:, 0] = (desired_positions[:, 0] * 0.84) +0.06
    desired_positions[:, 1] = (desired_positions[:, 1] * 0.6)+0.06
    # Check if any of the desired positions are within a manhatten distance of 2 of each other
    invalid = np.sum(np.abs(desired_positions[:, None, :] - desired_positions[None, :, :]), axis=-1) < 0.2
    np.fill_diagonal(invalid, False)
    invalid = np.any(invalid)
    while invalid:
        desired_positions = np.random.rand(num_shuttles, 2)
        desired_positions[:, 0] = (desired_positions[:, 0] * 0.84) +0.06
        desired_positions[:, 1] = (desired_positions[:, 1] * 0.6)+0.06
        invalid = np.sum(np.abs(desired_positions[:, None, :] - desired_positions[None, :, :]), axis=-1) < 0.2
        np.fill_diagonal(invalid, False)
        invalid = np.any(invalid)

    return desired_positions

class Shuttle(Node):

    def __init__(self):
        super().__init__('shuttle')
        # Define publishers and messages
        shuttle_config = load_yaml_file(self.get_name())
        print(shuttle_config)
        for key, value in shuttle_config.items():
            if "position" in key:
                self.declare_parameter(key)
                param_value = rclpy.Parameter(
                    key,
                    rclpy.Parameter.Type.DOUBLE_ARRAY,
                    value,
                )
                self.set_parameters([param_value])

        self.declare_parameter('num_of_shuttles')
        self.declare_parameter('sim_shuttle')
        self.number_of_shuttles = self.get_parameter('num_of_shuttles').get_parameter_value().integer_value
        sim_shuttle = self.get_parameter('sim_shuttle').get_parameter_value().bool_value
        self.get_logger().info('number of shuttles' + str(self.number_of_shuttles))

        if not sim_shuttle:
            self.number_of_shuttles = 0
        else:
            joint_names = ['x_translation', 'y_translation', 'z_translation', 'x_rotation', 'y_rotation', 'z_rotation']
            self.publisher_array = create_publisher_array(self, self.number_of_shuttles, topic_prefix=self.get_name(), topic_name='/joint_command', msg_type=JointState)
            self.msg_array = create_joint_state_message_array(joint_names, self.number_of_shuttles)

        # Define callback timer
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.mode = ShuttleMode()

    def shuttle_callback(self):
        try:
            self.number_of_shuttles = self.get_parameter('num_of_shuttles').get_parameter_value().integer_value
        except:
            pass



        # Update the amout of topics that there need to be
        if self.number_of_shuttles > len(self.publisher_array) or self.number_of_shuttles < len(self.publisher_array):
            destroy_publisher_array(self, self.publisher_array)
            self.publisher_array = create_publisher_array(self, self.number_of_shuttles, topic_prefix=self.get_name(), topic_name='/joint_command', msg_type=JointState)
            self.get_logger().info("Old shuttels destroyed")
        desired_positions = generate_random_goals(self.number_of_shuttles)
        for idx, publisher in enumerate(self.publisher_array):
            
            #pos = [0.06 + random.random()*0.84, 0.06 + random.random()*0.60, 0.0, 0.0, 0.0, 0.0]
            pos = [desired_positions[idx, 0], desired_positions[idx, 1], 0.0, 0.0, 0.0, 0.0]
            vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            msg = self.mode.move(pos, vel)
            publisher.publish(msg)

     


    
    
            

def main(args=None):
    """Initializes the Shuttle node and spins it until it is destroyed."""
    rclpy.init(args=args)

    shuttle_node = Shuttle()

    rclpy.spin(shuttle_node)
    #print("NU")
    #save_yaml(shuttle_node)
    shuttle_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
