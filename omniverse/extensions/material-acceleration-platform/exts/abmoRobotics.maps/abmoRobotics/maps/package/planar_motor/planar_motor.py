from ..base import Manipulator
from .controller import Controller, ForceFieldController
import numpy as np
import time
from typing import List
import copy

class PlanarMotor(Manipulator):

    def __init__(self, manipulator_root: str, manipulator_index: int) -> None:
        super().__init__(manipulator_root, manipulator_index)
        self.manipulator_type = 'planar_motor'
        self.manipulator_joint = self.manipulator_root + "/D6Joint"
        self.joint_names = ['transX', 'transY', 'transZ', 'rotX', 'rotY', 'rotZ']
        self.joint_target_position_attributes = \
            ['drive:%s:physics:targetPosition' % joint_name for joint_name in self.joint_names]
        self.joint_target_velocity_attributes = \
            ['drive:%s:physics:targetVelocity' % joint_name for joint_name in self.joint_names]

        self.position_for_controller = np.zeros(3)
        self.time_stamp_for_controller = time.time()
        self.velocity = np.zeros(3)
        self.joint_targets = np.zeros(6)
        self.previous_control_signal = np.zeros((5, 2))

    def set_joint_target(self, target_positions: list):
        self.joint_targets = np.array(target_positions)
        if not (len(target_positions) == len(self.joint_target_position_attributes)):
            raise AssertionError('Invalid number of joints')
        for attribute, target_position in zip(self.joint_target_position_attributes, target_positions):
            self.simulator._change_property(prim_path=self.manipulator_joint, attribute_name=attribute, value=target_position)

    def set_joint_target_relative(self, target_positions: list):
        if not (len(target_positions) == len(self.joint_target_position_attributes)):
            raise AssertionError('Invalid number of joints')
        for attribute, target_position in zip(self.joint_target_position_attributes, target_positions):
            target_position += self.simulator._get_property(prim_path=self.manipulator_joint, attribute=attribute)
            self.simulator._change_property(prim_path=self.manipulator_joint, attribute_name=attribute, value=target_position)

    def set_target_velocity(self, target_velocities: list):
        if not (len(target_velocities) == len(self.joint_target_velocity_attributes)):
            raise AssertionError('Invalid number of joints')
        for attribute, target_velocity in zip(self.joint_target_velocity_attributes, target_velocities):
            self.simulator._change_property(prim_path=self.manipulator_joint, attribute_name=attribute, value=target_velocity)

    def spawn(self):
        prim_path = self.manipulator_root
        asset_path = './assets/robots/shuttle_120x120.usd'
        self.simulator._create_payload(prim_path=prim_path, asset_path=asset_path)
        x = 0
        y = 0
        if self.manipulator_index == 0:
            x = 6.
            y = 6.
        elif self.manipulator_index == 1:
            x = 6.
            y = 90.
        elif self.manipulator_index == 2:
            x = 90.
            y = 6.
        elif self.manipulator_index == 3:
            x = 90.
            y = 90.
        self.set_initial_transform(x=x, y=y, z=1.0)
        #self.set_initial_transform(x=20 * self.manipulator_index + 6.0, y=6.0, z=1.0)

    def despawn(self):
        pass

    def set_initial_transform(self, x, y, z):
        print(str(x) + " " + str(y) + " " + str(z))
        prim_path = f'{self.manipulator_root}/shuttle'
        print("prim path" + prim_path)
        self.simulator._transform_prim(prim_path=prim_path, x=x, y=y, z=z)
        self.set_joint_target([x * 0.01, y * 0.01, z * 0.01, 0.0, 0.0, 0.0])

    def get_joint_position(self):
        position = np.array(self.simulator._get_property(prim_path=self.manipulator_root + "/shuttle", attribute='xformOp:translate') * 0.01) 
        return position

    def get_joint_velocity(self):
        if time.time() - self.time_stamp_for_controller > 0.01:
            current_position = self.get_joint_position()[0:3]
            self.velocity = (current_position - self.position_for_controller) / (time.time() - self.time_stamp_for_controller)
            self.position_for_controller = current_position
            self.time_stamp_for_controller = time.time()
        return self.velocity
    
    def get_joint_target(self):
        return self.joint_targets

    def apply_control_input(self):
        controller = Controller(p_gain=1, d_gain=0.1, max_veloicty=0.5)
    
        current_position = self.get_joint_position()
        current_velocity = self.get_joint_velocity()
        control_signal = -controller.get_control_signal(current_position=current_position[0:2], target_position=self.joint_targets[0:2], current_velocity=current_velocity[0:2])
        self.set_target_velocity([control_signal[0], control_signal[1], 0.0, 0.0, 0.0, 0.0])

    def apply_force_field_controller(self, current_positions, current_velocities, target_positions, potential_fields):
        
        controller = ForceFieldController(dt=0.015, timesteps=15, p_gain=2.5, d_gain=0.2, max_velocity=1) # timesteps = 15
        #current_position = self.get_joint_position()
        #current_velocity = self.get_joint_velocity()
        control_signal, repulsive_field = controller.predict_trajectory(current_position=current_positions[:,0:2], target_position=target_positions[:,0:2], current_velocity=current_velocities[:, 0:2], potential_field=potential_fields)
        
        for idx, (o, p) in enumerate(zip(control_signal, self.previous_control_signal)):
            angle = np.arccos(np.dot(o, p) / (np.linalg.norm(o) * np.linalg.norm(p)+0.0001))
            if angle > np.pi / 18 or angle < -np.pi / 18:
                control_signal[idx] = (o + p) / 2

        for idx, (o, p) in enumerate(zip(control_signal, self.previous_control_signal)):
            # Scale amplitude of control_signal to 90% or 110% of previous control_signal
            if np.linalg.norm(p) > 1.05:
                if np.linalg.norm(o) > 1.05 * np.linalg.norm(p):
                    factor = np.linalg.norm(o) / np.linalg.norm(p)
                    control_signal[idx] = o / factor * 1.05

                if np.linalg.norm(o) < 0.95 * np.linalg.norm(p):
                    factor = np.linalg.norm(o) / np.linalg.norm(p)
                    control_signal[idx] = o / factor * 0.95
        self.previous_control_signal = control_signal
        idx = self.manipulator_index
        #self.set_target_velocity([-control_signal[idx, 0], -control_signal[idx, 1], 0.0, 0.0, 0.0, 0.0])

        #controller2 = Controller(p_gain=1, d_gain=0.1, max_veloicty=0.5)
        # print(f'current_positions: {current_positions[:,0:2]}')
        # print(f'target_positions: {target_positions[:,0:2]}')
        # print(f'current_velocities: {current_velocities[:,0:2]}')
        #control_signal2 = controller2.get_control_signal(current_position=current_positions[:,0:2], target_position=target_positions[:,0:2], current_velocity=current_velocities[:,0:2])
        #print("ok")
        return (repulsive_field, control_signal)

# class ForceFieldManager:
#     def __init__(self, shuttles: List[PlanarMotor]) -> None:
#         self.shuttles = shuttles
#         self.potential_fields = np.zeros((len(self.shuttles), 2))
    
#     def calculate_force_field(self):
#         for idx, shuttle in enumerate(self.shuttles):
#             current_position = shuttle.get_joint_position()
#             current_velocity = shuttle.get_joint_velocity()
#             target_position = shuttle.get_joint_target()

#         controller = ForceFieldController(dt=0.01, timesteps=100, p_gain=5, d_gain=0.4, max_veloicty=2)



#             #self.potential_fields[idx] = self.calculate_repulsive_field(current_position, current_velocity, target_position)
#         #return self.potential_fields
    
    