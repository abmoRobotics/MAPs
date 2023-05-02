import rclpy
from typing import List, Any

import time
import json
import asyncio
import threading

import omni
import carb
import omni.kit
from pxr import Usd, Gf, PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.stage import get_stage_units
from rclpy.context import Context

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.parameter import Parameter

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

import omni.kit.commands
import pxr.Sdf as Sdf
import pxr.Usd as Usd


def acquire_maps_msgs_interface() -> Any:
    """TESTER FUNCTION."""
    global PoseSrv
    # import tutorial_interfaces  # .tutorial_interfaces
    from tutorial_interfaces.srv import PoseSrv as pose_srv

    PoseSrv = pose_srv

    try:
        rclpy.init()
    except:  # noqa E722
        print("ROS2 Already Initialized")

    bridge = ROS2Interface(PoseSrv)
    executer = rclpy.executors.MultiThreadedExecutor()
    executer.add_node(bridge)
    threading.Thread(target=executer.spin).start()
    return bridge


class ROS2Interface(Node):
    def __init__(self, msg_type):
        self._components = []
        super().__init__("tester_node")
        self.srv = self.create_service(msg_type, "spawn_robot", self.spawn_robot)
        self.srv = self.create_service(msg_type, "remove_robot", self.remove_robot)

        # omni objects and interfaces
        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage()
        self._timeline = omni.timeline.get_timeline_interface()
        self._physx_interface = omni.physx.acquire_physx_interface()
        self._dci = _dynamic_control.acquire_dynamic_control_interface()

        # events
        self._update_event = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update_event)
        self._timeline_event = self._timeline.get_timeline_event_stream().create_subscription_to_pop(self._on_timeline_event)
        self._stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)
        self._physx_event = self._physx_interface.subscribe_physics_step_events(self._on_physics_event)

        # variables
        self._manipulator_prim_path = "/World/tableScaled/Manipulators/"
        self._manipulator_asset_path = "./assets/robots/"
        self._manipulators_to_spawn = []
        self._manipulators_to_delete = []
        self._manipulators = []

    def spawn_robot(self, request, response):
        """SERVICE CALLBACK."""
        print(request)
        # response = True

        # create_payload(context=self._usd_context, prim_path=self._manipulator_prim_path, asset_path=self._manipulator_asset_path)
        self._manipulators_to_spawn.append({"name": request.name, "pose": request.pose})
        rate = self.create_rate(10)  # 10hz
        while self._manipulators_to_spawn:
            rate.sleep()
        response.success = True
        # response.data = True

        return response

    def remove_robot(self, request, response):
        """SERVICE CALLBACK."""
        self._manipulators_to_delete.append("test")
        rate = self.create_rate(10)  # 10hz
        while self._manipulators_to_delete:
            rate.sleep()

        response.success = True

        return response

    def _on_update_event(self, e: carb.events.IEvent):
        # pass
        if self._manipulators_to_spawn:
            for manipulator in self._manipulators_to_spawn:
                prim_path = self._manipulator_prim_path + manipulator["name"]
                asset_path = self._manipulator_asset_path + manipulator["name"][:-3] + ".usd"
                create_payload(context=self._usd_context, prim_path=prim_path, asset_path=asset_path)

                position = manipulator["pose"].position
                orientation = manipulator["pose"].orientation

                pos = Gf.Vec3d(position.x, position.y, position.z)
                quat = Gf.Quatd(orientation.w, orientation.x, orientation.y, orientation.z)

                if manipulator["name"][:-3] == "KR4R600":
                    # rotate 90 degrees about x axis
                    quat = quat * Gf.Quatd(0.707, 0.707, 0, 0)
                    scale = 0.001
                    matrix = convert_position_orientation_to_matrix(pos, quat, scale)
                    transform_prim(prim_path, matrix)

                elif manipulator["name"][:-3] == "KR3R540":
                    matrix = convert_position_orientation_to_matrix(pos, quat, scale=1.0)
                    transform_prim(prim_path, matrix)

                usd_prim = self._stage.GetPrimAtPath(prim_path + "/ActionGraph/ros2_subscribe_joint_state")
                print(usd_prim.GetAttribute("inputs:nodeNamespace"))
                usd_prim.GetAttribute("inputs:nodeNamespace").Set(manipulator["name"])
                self._manipulators_to_spawn.remove(manipulator)
        if self._manipulators_to_delete:
            for manipulator in self._manipulators_to_delete:
                self._stage.RemovePrim(self._manipulator_prim_path)
                # delete_payload(context=self._usd_context, prim_path=self._manipulator_prim_path)
                self._manipulators_to_delete.remove(manipulator)
        # # print("ok")

    def _on_timeline_event(self, e: carb.events.IEvent):
        pass

    def _on_stage_event(self, e: carb.events.IEvent):
        pass

    def _on_physics_event(self, e: carb.events.IEvent):
        pass

    def _stop_components(self) -> None:
        """Stop all components."""
        for component in self._components:
            component.stop()

    def shutdown(self) -> None:
        """Shutdown the ROS2Interface inteface."""
        self._update_event = None
        self._timeline_event = None
        self._stage_event = None

        self._stop_components()
        self.destroy_node()
        rclpy.shutdown()


def destroy_maps_msgs_interface(ROS2Interface: ROS2Interface) -> None:
    """
    Release the Ros2Bridge interface.

    :param bridge: The Ros2Bridge interface
    :type bridge: Ros2Bridge
    """
    ROS2Interface.shutdown()


def create_payload(context: omni.usd._usd.UsdContext, prim_path: str, asset_path: str):
    """Create a payload."""
    omni.kit.commands.execute("CreatePayload", usd_context=context, path_to=Sdf.Path(prim_path), asset_path=asset_path, instanceable=False)
    print("Created Payload")


def delete_prims(context: omni.usd._usd.UsdContext, prim_path: str):
    """Delete prims."""
    omni.kit.commands.execute("DeletePrims", paths=[Sdf.Path(prim_path)], destructive=False)


def transform_prim(prim_path: str, matrix: Gf.Matrix3d):
    """Transform a prim."""
    omni.kit.commands.execute(
        "TransformPrim",
        path=Sdf.Path(prim_path),
        new_transform_matrix=matrix,
    )


def convert_position_orientation_to_matrix(position: Gf.Vec3d, orientation: Gf.Quatd, scale: float) -> Gf.Matrix4d:
    """Convert position and orientation to a matrix."""
    # Create a translation matrix from the position
    translation_matrix = Gf.Matrix4d().SetTranslate(position)

    # Create a rotation matrix from the orientation
    rotation_matrix = Gf.Matrix4d().SetRotate(orientation)

    # Create a scale matrix to scale the object
    scale_matrix = Gf.Matrix4d().SetScale(Gf.Vec3d(scale, scale, scale))

    # Combine the translation, rotation, and scale matrices
    transformation_matrix = scale_matrix * rotation_matrix * translation_matrix

    return transformation_matrix
