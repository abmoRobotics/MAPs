import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('shuttles')
        self.publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        thread = threading.Thread(target=rclpy.sp)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


        joint_state_position = JointState()
        joint_state_velocity = JointState()

        joint_state_position.name = ["joint1", "joint2","joint3"]
        joint_state_velocity.name = ["wheel_left_joint", "wheel_right_joint"]
        joint_state_position.position = [0.2,0.2,0.2]
        joint_state_velocity.velocity = [20.0, -20.0]





    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    thread = threading.Thread(target=rclpy.spin(minimal_publisher), args=(minimal_publisher, ), daemon=True)
    thread.start()



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()