import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading

from std_msgs.msg import String

def no_of_robots(self, number_of_robot):
        number_of_robot = int(number_of_robot)
        self.publisher = []
        for i in range(0,number_of_robot):
            self.get_logger().info('Publishing' + str(i))
            self.publisher.append(self.create_publisher(JointState, '/shuttle'+str(i)+'/joint_command', 10))



class MinimalPublisher(Node):


    def __init__(self):
        super().__init__('shuttles')
        #self.publisher_ = self.create_publisher(JointState, 'shuttle_'+str(1)+'/joint_command', 10)
        no_of_robots(self, 5)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


        self.msg = JointState()

        self.msg.name = ["lin_X", "lin_Y","lin_Z", "ang_X", "ang_Y", "ang_Z","LinVel_X", "LinVel_Y", "LinVel_Z","AngVel_X", "AngVel_Y", "AngVel_Z"]
        self.msg.position = [0.2,0.2,0.2]
        self.msg.velocity = [20.0, -20.0]





    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1
        self.get_logger().info('Publishing')
        self.publisher[1].publish(self.msg)
        


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