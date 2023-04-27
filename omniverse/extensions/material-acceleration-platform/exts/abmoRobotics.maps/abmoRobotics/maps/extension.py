import omni.ext
import carb.events
import omni.kit.app
import time
import rclpy
from .ros2.manager import RosManager
# import omni.ui as ui


# Functions and vars are available to other extension as usual in python: `example.python_ext.some_public_function(x)`
def some_public_function(x: int):
    print("[abmoRobotics.maps] some_public_function was called with x: ", x)
    return x ** x


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class AbmoroboticsMapsExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
   # update_stream = omni.kit.app.get_app().get_update_event_stream()
    def initialize(self):
        self.dummy_node = rclpy.create_node('dummy')

    def on_startup(self, ext_id):
        print("[abmoRobotics.maps] abmoRobotics maps startup")
        self.update_stream = omni.kit.app.get_app().get_update_event_stream()
        self.sub = self.update_stream.create_subscription_to_pop(self.on_event, name="abmoRobotics.maps")
        self.last_time = time.perf_counter()
        #self.initialize()
        ros_manager = RosManager()
        ros_manager.check_for_new_shuttle_topics()
        # self._count = 0

        # self._window = ui.Window("My Window", width=300, height=300)
        # with self._window.frame:
        #     with ui.VStack():
        #         label = ui.Label("")


        #         def on_click():
        #             self._count += 1
        #             label.text = f"count: {self._count}"

        #         def on_reset():
        #             self._count = 0
        #             label.text = "empty"

        #         on_reset()

        #         with ui.HStack():
        #             ui.Button("Add", clicked_fn=on_click)
        #             ui.Button("Reset", clicked_fn=on_reset)

    def on_shutdown(self):
        print("[abmoRobotics.maps] abmoRobotics maps shutdown")

    def on_event(self, e: carb.events.IEvent):
        #print("working")
        # Calculate dt
        dt = time.perf_counter() - self.last_time
        self.last_time = time.perf_counter()
        # Convert to fps
        fps = 1.0 / dt
        #print(f'FPS: {fps}')
        #shuttle_topics = [x[0] for x in self.dummy_node.get_topic_names_and_types() if ("shuttle" in x[0] and x[0].endswith("joint_command") and "JointState" in x[1][0])]
        #print(e)

