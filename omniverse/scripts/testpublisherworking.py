import asyncio
import rclpy
from rclpy.node import Node

from std_msgs.msg import String



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


async def my_task():
	rclpy.init()
	minimal_subscriber = MinimalSubscriber()
	for x in range(100):
		rclpy.spin_once(minimal_subscriber, timeout_sec=0.001)
		await asyncio.sleep(0.1)
	print("hej")

asyncio.ensure_future(my_task())
print("hej2")
rclpy.shutdown()