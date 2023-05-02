import omni.kit.commands

from pxr import Gf, Sdf


class Manipulator:
    def __init__(self, manipulator_root: str, manipulator_index: int) -> None:
        self.manipulator_root = f"{manipulator_root}{manipulator_index:02}"
        self.manipulator_index = manipulator_index
        self.simulator = OmniverseInterface()

    def set_joint_target(set_target_position: list):
        raise NotImplementedError("Method not implemeted")

    def set_joint_target_relative(self, target_positions: list):
        raise NotImplementedError("Method not implemented")

    def set_target_velocity(self, target_velocities: list):
        raise NotImplementedError("Method not implemented")

    def spawn(self):
        raise NotImplementedError("Method not implemented")

    def despawn(self):
        raise NotImplementedError("Method not implemented")

    def set_initial_transform(self, x, y, z):
        raise NotImplementedError("Method not implemented")


class SixAxisRobotInterace():
    def __init__(self, manipulator_root: str, manipulator_index: int) -> None:
        self.manipulator_root = f"{manipulator_root}_{manipulator_index:02}"
        self.manipulator_index = manipulator_index
        self.omni_interface = OmniverseInterface()

    def change_node_namespace(self, namespace: str):
        raise NotImplementedError("Method not implemented")

    def change_topic(self, topic: str):
        raise NotImplementedError("Method not implemented")

    def spawn(self):
        raise NotImplementedError("Method not implemented")

    def _set_initial_transform(self, x, y, z):
        raise NotImplementedError("Method not implemented")


class OmniverseInterface:
    def __init__(self) -> None:
        self.stage = omni.usd.get_context().get_stage()

    def _change_property(self, prim_path: str, attribute_name: str, value: float):
        prim = self.stage.GetPrimAtPath(prim_path)
        prim.GetAttribute(attribute_name).Set(value)

    def _get_property(self, prim_path: str, attribute: str):
        prim = self.stage.GetPrimAtPath(prim_path)
        prim_property = prim.GetAttribute(attribute)
        return prim_property.Get()

    def _create_payload(self, prim_path: str, asset_path: str):
        omni.kit.commands.execute('CreatePayload',
                                  usd_context=omni.usd.get_context(),
                                  path_to=Sdf.Path(prim_path),
                                  asset_path=asset_path,
                                  instanceable=False)

    def _transform_prim(self, prim_path: str, x: float, y: float, z: float):
        print(Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                          0.0, 1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0, 0.0,
                          x, y, z, 1.0))
        omni.kit.commands.execute('TransformPrim',
                                  path=Sdf.Path(prim_path),
                                  new_transform_matrix=Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                                                                   0.0, 1.0, 0.0, 0.0,
                                                                   0.0, 0.0, 1.0, 0.0,
                                                                   x, y, z, 1.0))
