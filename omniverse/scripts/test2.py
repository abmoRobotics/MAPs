import asyncio
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
async def main():
    #rclpy.init(args=None)
    #node = rclpy.create_node('minimal_publisher')

    #publisher = node.create_publisher(String, 'topic', 10)

    task = asyncio.ensure_future(other_function())
    print("A")
    #await asyncio.sleep(1)
    print("B")

    #await task

async def other_function():
    print("1")
    time.sleep(2)
    print(2)
    # msg = String()
    # send = True
    # i = 0
    # while rclpy.ok():
    #     msg.data = 'Hello World: %d' % i
    #     i += 1
    #     node.get_logger().info('Publishing: "%s"' % msg.data)
    #     if send:
    #         publisher.publish(msg)
    #         send= False
    #     time.sleep(0.5)  # seconds
        #rclpy.spin(minimal_publisher)
# func1()
# func2()
# func3()

asyncio.run(main())
print("hej")
time.sleep(3)