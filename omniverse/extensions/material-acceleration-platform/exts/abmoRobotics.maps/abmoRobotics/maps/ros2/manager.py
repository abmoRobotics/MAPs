import rclpy
import pxr.Usd as Usd
import pxr.Sdf as Sdf
from .subscribers import JointStateSubscriber, InitialPoseSubscriber
from ..package.planar_motor import PlanarMotor
import omni.usd
import omni.kit.commands
from typing import List
import numpy as np


class RosManager:
    def __init__(self) -> None:
        self.joint_state_subscribers: List[JointStateSubscriber] = []
        self.initial_pose_subscribers: List[InitialPoseSubscriber] = []
        self.shuttles: List[PlanarMotor] = []
        self.shuttle_prefix = "/World/tableScaled/joints/shuttle_120x120_"  # Format _ + f'{idx:02}'
        self.stage = omni.usd.get_context().get_stage()

        self.initialize_ros()
        print("ROS initialized")

    def initialize_ros(self):
        if not rclpy.utilities.ok():
            rclpy.init()
        self.dummy_node = rclpy.create_node("dummy_node")

    def reset_manager(self):
        self.joint_state_subscribers = []
        self.initial_pose_subscribers = []
        self.shuttles = []

    def add_subscriber(self, topic, prim_path):
        self.joint_state_subscribers.append(JointStateSubscriber(topic, prim_path))
        self.initial_pose_subscribers.append(InitialPoseSubscriber(topic, prim_path))

    def add_shuttle_prim(self, shuttle):
        self.shuttles.append(shuttle)

    def check_for_new_shuttle_topics(self):
        shuttle_topics = [
            x[0] for x in self.dummy_node.get_topic_names_and_types() if ("shuttle" in x[0] and x[0].endswith("joint_command"))
        ]
        for idx, topic in enumerate(shuttle_topics):
            sdf_path = Sdf.Path(f"{self.shuttle_prefix}{idx:02}")
            prim: Usd.Prim = self.stage.GetPrimAtPath(sdf_path)
            if bool(self.dummy_node.get_publishers_info_by_topic(topic)):
                if not prim.IsValid():
                    planar_motor = PlanarMotor(self.shuttle_prefix, idx)
                    planar_motor.spawn()
                    self.add_shuttle_prim(planar_motor)
                    self.add_subscriber(topic, planar_motor)
            else:
                if prim.IsValid():
                    omni.kit.commands.execute("DeletePrims", paths=[Sdf.Path(f"{self.shuttle_prefix}{idx:02}")], destructive=False)

    def _apply_control_input(self, control_inputs):
        for shuttle, control_input in zip(self.shuttles, control_inputs):
            control_input = [-control_input[0], -control_input[1], 0.0, 0.0, 0.0, 0.0]
            shuttle.set_target_velocity(control_input)

    def _get_current_shuttle_states(self):
        velocities = np.zeros((len(self.shuttles), 3))
        positions = np.zeros((len(self.shuttles), 3))
        target_positions = np.zeros((len(self.shuttles), 6))

        for idx, shuttle in enumerate(self.shuttles):
            velocities[idx] = shuttle.get_joint_velocity()
            positions[idx] = shuttle.get_joint_position()
            target_positions[idx] = shuttle.get_joint_target()

        return velocities, positions, target_positions

    def calculate_and_apply_control_input(self):
        velocities, positions, target_positions = self._get_current_shuttle_states()
        potentials = np.zeros((len(self.shuttles), 2))

        if self.check_for_collisions(positions):
            print("Collision detected")

        try:
            self.potential_field, control_signal = self.shuttles[0].apply_force_field_controller(
                positions, velocities, target_positions, potentials
            )
            self._apply_control_input(control_signal)
        except Exception as e:
            print(f"Err: {e}")

        return potentials

    def check_for_collisions(self, positions):
        distances = self.calculate_distances(positions)
        return self.check_threshold(distances, 0.121)

    def calculate_distances(self, points):
        points = np.array(points)
        diff = points[:, np.newaxis, :] - points[np.newaxis, :, :]
        distances = np.max(np.abs(diff), axis=-1)
        return distances

    def check_threshold(self, distances, threshold):
        np.fill_diagonal(distances, np.inf)  # Set diagonal elements to infinity to ignore self-distances
        return np.any(distances < threshold)
