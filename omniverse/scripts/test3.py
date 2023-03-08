import threading
import asyncio
import rclpy
from rclpy.node import Node

async def spin(node: Node):
    cancel = node.create_guard_condition(lambda: None)
    def _spin(node: Node,
              future: asyncio.Future,
              event_loop: asyncio.AbstractEventLoop):
        while not future.cancelled():
            rclpy.spin_once(node)
        if not future.cancelled():
            event_loop.call_soon_threadsafe(future.set_result, None)
    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_future()
    spin_thread = threading.Thread(target=_spin, args=(node, spin_task, event_loop))
    spin_thread.start()
    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()
    spin_thread.join()
    node.destroy_guard_condition(cancel)

async def main():
    # create a node without any work to do
    node = Node('unused')

    # create tasks for spinning and sleeping
    spin_task = asyncio.get_event_loop().create_task(spin(node))
    sleep_task = asyncio.get_event_loop().create_task(asyncio.sleep(5.0))

    # concurrently execute both tasks
    await asyncio.wait([spin_task, sleep_task], return_when=asyncio.FIRST_COMPLETED)

    # cancel tasks
    if spin_task.cancel():
        await spin_task
    if sleep_task.cancel():
        await sleep_task

# run program
rclpy.init()
print("hej")
asyncio.get_event_loop().run_until_complete(main())
asyncio.get_event_loop().close()
rclpy.shutdown()