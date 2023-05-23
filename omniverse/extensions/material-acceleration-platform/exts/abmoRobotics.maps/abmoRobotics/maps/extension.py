import omni.ext
import carb.events
import pxr.Usd as Usd
import pxr.Sdf as Sdf
import omni.kit.app
import time
import rclpy
from .ros2.manager import RosManager
import omni.ui as ui
import omni.kit.commands
import omni.usd
from sensor_msgs.msg import JointState

# import pxr.Usd
# import pxr.Sdf


# Functions and vars are available to other extension as usual in python: `example.python_ext.some_public_function(x)`
def some_public_function(x: int):
    print("[abmoRobotics.maps] some_public_function waas called wcith x: ", x)
    return x**x


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class AbmoroboticsMapsExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def initialize(self):
        self.dummy_node = rclpy.create_node("dummy")

    def on_startup(self, ext_id):
        self.ros_started = False
        print("[abmoRobotics.maps] abmoRobotics mas startup")
        self.update_stream = omni.kit.app.get_app().get_update_event_stream()
        self.sub = self.update_stream.create_subscription_to_pop(self.on_event, name="abmoRobotics.maps")
        self.last_time = time.perf_counter()
        self.stage: Usd.Stage = omni.usd.get_context().get_stage()
        self.ros_manager = RosManager()

        self._count = 1

        self._window = ui.Window("MAPs ", width=300, height=100)
        with self._window.frame:
            with ui.VStack():
                label = ui.Label("")

                def on_reset():
                    self.stage: Usd.Stage = omni.usd.get_context().get_stage()
                    for i in range(50):
                        sdf_path = Sdf.Path(f"/World/tableScaled/joints/shuttle_120x120_{i:02}")
                        prim: Usd.Prim = self.stage.GetPrimAtPath(sdf_path)

                        if prim.IsValid():
                            try:
                                self.stage.RemovePrim(prim.GetPath())
                                self.ros_manager.reset_manager()
                                self.ros_started = False
                            except Exception as e:
                                print(f"Error deleting prim: {e}")
                    label.text = "Not runnng"

                def on_click():
                    self._count += 1
                    # label.text = f"count: {self._count}"
                    self.start_ros()
                    label.text = "Status: Running"

                # on_reset()

                with ui.HStack():
                    ui.Button("Start", clicked_fn=on_click)
                    ui.Button("Reset", clicked_fn=on_reset)

    def start_ros(self):
        try:
            print("starting ros")
            self.ros_manager.check_for_new_shuttle_topics()
            self.ros_started = True
        except Exception as e:
            print(f"Error starting ROS: {e}")

    def on_shutdown(self):
        print("[abmoRobotics.maps] abmoRobotics maaps shutdown")

    def on_event(self, e: carb.events.IEvent):
        # if not self.ros_started:
        #     self.start_ros()

        dt = time.perf_counter() - self.last_time
        self.last_time = time.perf_counter()
        fps = 1.0 / dt

        # print(f)
        if self.ros_started:
            if self.ros_manager.shuttles[0] is not None:
                self.ros_manager.calculate_and_apply_control_input()
            for joint_state_subscriber in self.ros_manager.joint_state_subscribers:
                rclpy.spin_once(joint_state_subscriber, timeout_sec=0.00000000000000001)
            for initial_pose_subscriber in self.ros_manager.initial_pose_subscribers:
                rclpy.spin_once(initial_pose_subscriber, timeout_sec=0.00000000000000001)

            velocities, positions, target_positions = self.ros_manager._get_current_shuttle_states()

            for idx, joint_state_publisher in enumerate(self.ros_manager.joint_state_publisher):
                joint_state = JointState()
                joint_state.name.append(f"{idx}")
                for i in range(3):
                    joint_state.position.append(float(positions[idx][i]))
                    joint_state.velocity.append(float(velocities[idx][i]))
                joint_state_publisher.publish_joint_state(joint_state)
