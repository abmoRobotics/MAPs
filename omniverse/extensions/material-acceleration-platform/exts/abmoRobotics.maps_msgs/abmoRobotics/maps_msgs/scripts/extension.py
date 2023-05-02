import omni.ext
import os
import sys
import carb
import rclpy

# import srv.srv._pose_srv as pose_srv
# from .srv_test import srv

try:
    from .. import _custom_msgs
except Exception as e:  # noqa E722
    print(">>>> [DEVELOPMENT] imort custom_msgs")
    from .. import custom_msgs as _custom_msgs

# from .. import custom_msgs as _custom_msgs


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.bridge = None
        self._extension_path = None
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        if ext_manager.is_extension_enabled("omni.isacc.ros_bridge"):
            carb.log("ROS2 Bridge external extension cannot be enabled if ROS Bridge is enabled")
            ext_manager.disable_extension("abmoRobotics.maps_msgs")
            return

        self._extension_path = ext_manager.get_extension_path(ext_id)
        sys.path.append(os.path.join(self._extension_path, "abmoRobotics", "maps_msgs", "packages"))

        if os.environ.get("LD_LIBRARY_PATH"):
            os.environ["LD_LIBRARY_PATH"] = os.environ.get("LD_LIBRARY_PATH") + ":{}/bin".format(self._extension_path)
        else:
            os.environ["LD_LIBRARY_PATH"] = "{}/bin".format(self._extension_path)

        self.bridge = _custom_msgs.acquire_maps_msgs_interface()

    def on_shutdown(self):
        print("o2ka21")
        if self._extension_path is not None:
            sys.path.remove(os.path.join(self._extension_path, "abmoRobotics", "maps_msgs", "packages"))
            self._extension_path = None

        if self.bridge is not None:
            try:
                _custom_msgs.destroy_maps_msgs_interface(self.bridge)
                self.bridge = None
            except Exception as e:
                print("EXCEPTION SHUTDOWN")
                print(e)
