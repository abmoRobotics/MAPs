import asyncio
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


async def tester(pub):
    msg = String()
    msg.data = "Hello World"
    for i in range(10):
        print(i)
        pub.publish(msg)
        await asyncio.sleep(1.0)
    print("ok")
    pub.unregister()
    pub = None


node = rclpy.create_node("inline_publisher")
publisher = node.create_publisher(String, "topicer", 10)
asyncio.ensure_future(tester(publisher))


### Version 2

import asyncio
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


async def tester(pub):
    msg = String()
    msg.data = "Hello World"
    while rclpy.ok():
        pub.publish(msg)
        await asyncio.sleep(0.00000001)
    print("ok")
    pub.unregister()
    pub = None


node = rclpy.create_node("inline_publisher")
publisher = node.create_publisher(String, "topicer", 10)
asyncio.ensure_future(tester(publisher))

### Service

import rclpy
from std_srvs.srv import SetBool
import asyncio


async def toggle_led(request, response):
    global led_state
    led_state = request.data
    response.success = True
    response.message = "LED toggled"
    return response


async def service_test():
    global led_state
    node = rclpy.create_node("service_test")

    srv = node.create_service(SetBool, "/toggle_led", toggle_led)

    for i in range(10):
        print(i)
        rclpy.spin_once(node)
        await asyncio.sleep(0.5)

    node.destroy_node()


asyncio.ensure_future(service_test())
