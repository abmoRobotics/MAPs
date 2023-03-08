import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


from utils import create_joint_state_message_array, create_publisher_array, destroy_publisher_array


class GUI(Node):

    def __init__(self):
        super().__init__('gui')
        self.declare_parameters(namespace='',parameters=[('shuttle1_position', None),
                                                         ('shuttle2_position', None),
                                                         ('shuttle3_position', None),
                                                         ('shuttle4_position', None),
                                                         ('shuttle5_position', None),
                                                         ('num_of_shuttles', None)])
        
        number_of_shuttles = self.get_parameter('num_of_shuttles').get_parameter_value().integer_value
        self.get_logger().info(str(self.get_parameter('shuttle1_position').get_parameter_value().double_array_value))

        en = self.get_parameter('shuttle1_position').get_parameter_value().double_array_value
        to = self.get_parameter('shuttle2_position').get_parameter_value().double_array_value
        tre = self.get_parameter('shuttle3_position').get_parameter_value().double_array_value
        fire = self.get_parameter('shuttle4_position').get_parameter_value().double_array_value
        fem = self.get_parameter('shuttle5_position').get_parameter_value().double_array_value
        
        self.msg = JointState()
        param = [en, to, tre, fire, fem]

        # Define callback timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        joint_names = ['x_translation', 'y_translation', 'z_translation', 'x_rotation', 'y_rotation', 'z_rotation']
        publisher_array= create_publisher_array(self, number_of_shuttles,'/initialPosition')
        self.msg_array = create_joint_state_message_array(joint_names, number_of_shuttles)


        for idx, publisher in enumerate(publisher_array):
            self.msg.position = param[idx]
            publisher.publish(self.msg)
        

    def timer_callback(self):
        self.get_logger().info(str(self.get_parameter('shuttle1_position').get_parameter_value().double_array_value))





def main(args=None):
    """Initializes the gui node and spins it until it is destroyed."""
    rclpy.init(args=args)

    gui_node = GUI()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
