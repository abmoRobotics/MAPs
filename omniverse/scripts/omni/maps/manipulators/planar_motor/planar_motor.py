from ..base import Manipulator

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

    def set_target_position(self, target_positions: list):
        if not (len(target_positions) == len(self.joint_target_position_attributes)):
            raise AssertionError('Invalid number of joints')
        for attribute, target_position in zip(self.joint_target_position_attributes, target_positions):
            self.change_property(prim_path=self.manipulator_joint, attribute_name=attribute, value=target_position)
    
    def set_target_position_relative(self, target_positions: list):
        if not (len(target_positions) == len(self.joint_target_position_attributes)):
            raise AssertionError('Invalid number of joints')
        for attribute, target_position in zip(self.joint_target_position_attributes, target_positions):
            target_position += self.get_property(prim_path=self.manipulator_joint, attribute=attribute)
            self.change_property(prim_path=self.manipulator_joint, attribute_name=attribute, value=target_position)

    def set_target_velocity(self, target_velocities: list):
        if not (len(target_velocities) == len(self.joint_target_velocity_attributes)):
            raise AssertionError('Invalid number of joints')
        for attribute, target_velocity in zip(self.joint_target_velocity_attributes, target_velocities):
            self.change_property(prim_path=self.manipulator_joint, attribute_name=attribute, value=target_velocity)
    
    def spawn_shuttle(self):
        prim_path = self.manipulator_root
        asset_path = './assets/robots/shuttle_120x120.usd'
        self.create_payload(prim_path=prim_path, asset_path=asset_path)
        self.set_initial_transform(x=20*self.manipulator_index+6.0, y=6.0, z=1.0)

    def set_initial_transform(self, x, y, z):
        print(str(x) + " " + str(y) + " " + str(z))
        prim_path = f'{self.manipulator_root}/shuttle'
        print("prim path" + prim_path)
        self.transform_prim(prim_path=prim_path, x=x, y=y, z=z)
        self.set_target_position([x*0.01, y*0.01, z*0.01, 0.0, 0.0, 0.0])
