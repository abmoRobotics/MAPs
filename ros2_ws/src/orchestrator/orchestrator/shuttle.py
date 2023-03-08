import rclpy
from rclpy.node import Node

from utils import create_joint_state_message_array, create_publisher_array, destroy_publisher_array
from utils import ShuttleMode
from sensor_msgs.msg import JointState
class Shuttle(Node):

    def __init__(self):
        super().__init__('shuttle')
        # Define publishers and messages
        self.declare_parameter('num_of_shuttles')
        self.declare_parameter('sim_shuttle')
        self.number_of_shuttles = self.get_parameter('num_of_shuttles').get_parameter_value().integer_value
        sim_shuttle = self.get_parameter('sim_shuttle').get_parameter_value().bool_value
        self.get_logger().info("number of shuttles" +  str(self.number_of_shuttles))
        
        if sim_shuttle == False:
            self.number_of_shuttles = 0
        else:
            joint_names = ['x_translation', 'y_translation', 'z_translation', 'x_rotation', 'y_rotation', 'z_rotation']
            self.publisher_array = create_publisher_array(self, self.number_of_shuttles, topic_prefix=self.get_name(), topic_name='/joint_command', msg_type=JointState)
            self.msg_array = create_joint_state_message_array(joint_names, self.number_of_shuttles)

        # Define callback timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.mode = ShuttleMode()

    def timer_callback(self):
        self.number_of_shuttles = self.get_parameter('num_of_shuttles').get_parameter_value().integer_value



        # Update the amout of topics that there need to be
        if self.number_of_shuttles > len(self.publisher_array) or self.number_of_shuttles < len(self.publisher_array):
            destroy_publisher_array(self, self.publisher_array)
            self.publisher_array = create_publisher_array(self, self.number_of_shuttles, topic_prefix=self.get_name(), topic_name='/joint_command', msg_type=JointState)
            self.get_logger().info("Old shuttels destroyed")

        for idx, publisher in enumerate(self.publisher_array):
            pos = [(0.15)*(idx)+0.1, -1.0, 0.0, 0.0, 0.0, 0.0]
            vel = [(0.15)*(idx)+0.1, 2.0, 0.0, 0.0, 0.0, 0.0]
            msg = self.mode.move(pos,vel)
            publisher.publish(msg)

     


    
    
            

def main(args=None):
    """Initializes the Shuttle node and spins it until it is destroyed."""
    rclpy.init(args=args)

    shuttle_node = Shuttle()

    rclpy.spin(shuttle_node)

    shuttle_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
