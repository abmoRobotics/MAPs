import rclpy
from rclpy.node import Node


from utils import create_joint_state_message_array, create_publisher_array, destroy_publisher_array, load_yaml_file, save_yaml, create_action_server_array
from utils import ShuttleMode
import time
from sensor_msgs.msg import JointState
import random
import numpy as np
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterDescriptor, ParameterValue, Parameter
from rclpy.parameter import ParameterType
from robot_actions.action import Shu
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor

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


        self.declare_parameter('sim_shuttle')
        sim_shuttle = self.get_parameter('sim_shuttle').get_parameter_value().bool_value
        
        # Define the number of shuttles that is from start
        self.num_shu = 0
        self.client = self.create_client(GetParameters,
                                        '/global_parameter_server/get_parameters')

        self.shuttle_positions = np.zeros((100, 2))
        self.shuttle_subscriber_array = []
        if not sim_shuttle:
            self.num_shu = 0
        else:
            joint_names = ['x_translation', 'y_translation', 'z_translation', 'x_rotation', 'y_rotation', 'z_rotation']
            self.publisher_array = create_publisher_array(self, self.num_shu, topic_prefix=self.get_name(), topic_name='/joint_command', msg_type=JointState)
            self.msg_array = create_joint_state_message_array(joint_names, self.num_shu)
            for idx in range(self.num_shu):
                self.shuttle_subscriber_array.append(self.create_subscription(JointState, '/' + self.get_name() + f'{idx:02}' + '/joint_states', self.shuttle_joint_state_callback, 10))
            
            self.action_server_array = create_action_server_array(self, Shu, topic_prefix=self.get_name(), callback_type=self.action_shuttle_callback, n_robots=self.num_shu)
    
        # Define callback timer
        timer_period = 1  # seconds
        timer_period_states = 0.1
        self.timer = self.create_timer(timer_period, self.shuttle_callback)
        self.timer_states = self.create_timer(timer_period_states, self.update_states)

        self.mode = ShuttleMode()

    def update_states(self):
                # Request parameter from the /gui node
        request = GetParameters.Request()
        request.names = ['num_of_shuttles']
        future = self.client.call_async(request=request)
        future.add_done_callback(self.callback_global_param)

    def shuttle_callback(self):
        # # try:
        # #     self.get_logger().info(f'Shuttle positions {self.shuttle_positions}')
        # # except:
        # #     pass


        # Update the amout of topics that there need to be
        if self.num_shu> len(self.publisher_array) or self.num_shu < len(self.publisher_array):
            destroy_publisher_array(self, self.publisher_array)
            self.publisher_array = create_publisher_array(self, self.num_shu, topic_prefix=self.get_name(), topic_name='/joint_command', msg_type=JointState)
            #self.action_server_array = create_action_server_array(self, Shu, topic_prefix=self.get_name(), callback_type=self.action_shuttle_callback, n_robots=self.num_shu)
            self.get_logger().info("Old shuttels destroyed")
            self.shuttle_subscriber_array = []
            for action_server in self.action_server_array:
                action_server.destroy()
            self.action_server_array = []

            for idx in range(self.num_shu):
                self.action_server_array.append(ActionServer(self, Shu, self.get_name() + f'{idx:02}', self.action_shuttle_callback))

            for shuttle in self.shuttle_subscriber_array:
                shuttle.destroy()
            self.shuttle_subscriber_array = []
            for idx in range(self.num_shu):
                self.shuttle_subscriber_array.append(self.create_subscription(JointState, '/' + self.get_name() + f'{idx:02}' + '/joint_states', self.shuttle_joint_state_callback, 10))
        desired_positions = generate_random_goals(self.num_shu)
        # for idx, publisher in enumerate(self.publisher_array):
            
        #     #pos = [0.06 + random.random()*0.84, 0.06 + random.random()*0.60, 0.0, 0.0, 0.0, 0.0]
        #     pos = [desired_positions[idx, 0], desired_positions[idx, 1], 0.0, 0.0, 0.0, 0.0]
        #     vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #     msg = self.mode.move(pos, vel)
        #     publisher.publish(msg)

    def action_shuttle_callback(self, goal_handle):
        self.get_logger().info("Goal request" + str(goal_handle.request.goal))

        goal = goal_handle.request.goal
        shuttle_id = goal_handle.request.id
        target_position = goal

        target_position = np.array(target_position)
        # Loop until condition is met
        # self.get_logger().info("Shuttle " + str(shuttle_id) + " is moving to " + str(target_position))
        #self.get_logger().info(f'{self.shuttle_positions[shuttle_id]}')
        # self.get_logger().info(f'{np.linalg.norm(target_position[0:2] - self.shuttle_positions[shuttle_id])}')
        self.get_logger().info(f'action shuttle id {shuttle_id}, target position {target_position}')
        while np.linalg.norm((target_position[0:2] - self.shuttle_positions[shuttle_id])) > 0.05:
            self.get_logger().info(f'distance {np.linalg.norm((target_position[0:2] - self.shuttle_positions[shuttle_id]))}')
            #self.get_logger().info(f'current position {self.shuttle_positions[shuttle_id]}')
            #self.get_logger().info(f'target position {target_position[0:2]}')
            pos = [target_position[0], target_position[1], 0.0, 0.0, 0.0, 0.0]
            vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            msg = self.mode.move(pos, vel)
            self.publisher_array[shuttle_id].publish(msg)
            time.sleep(0.1)
        self.get_logger().info("goal reached sleeping")
        time.sleep(5)
        self.get_logger().info("goal reached")

        result = Shu.Result()
        result.goalresult = "Success"
        result.id = shuttle_id

        return result






     
    def callback_global_param(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn("Service call failed inside shuttle %r" % (e,))
        else:
            self.param = result.values[0]
            self.num_shu = self.param.integer_value
            #self.get_logger().info("Number of shuttle is: %s" % (self.num_shu,))


    def shuttle_joint_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        idx = int(msg.name[0])
        # self.get_logger().info(f'shuttle joint state idx {idx}')
        x, y = msg.position[0], msg.position[1]
        position = np.array([x, y])
        self.shuttle_positions[idx] = position


    
    
            

def main(args=None):
    """Initializes the Shuttle node and spins it until it is destroyed."""
    rclpy.init(args=args)

    shuttle_node = Shuttle()

    # rclpy.spin(shuttle_node)
    # #print("NU")
    # #save_yaml(shuttle_node)
    # shuttle_node.destroy_node()
    # rclpy.shutdown()


    executor = MultiThreadedExecutor()
    executor.add_node(shuttle_node)
    try:
        executor.spin()
    finally:
        # Shutdown and remove node
        executor.shutdown()
        shuttle_node.destroy()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
