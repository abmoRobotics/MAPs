import omni.kit.commands

from pxr import Gf, Sdf


class Manipulator:
    def __init__(self, manipulator_root: str, manipulator_index: int) -> None:
        self.stage = omni.usd.get_context().get_stage()
        self.manipulator_root = f"{manipulator_root}_{manipulator_index}"
        self.manipulator_index = manipulator_index

    def change_property(self, prim_path: str, attribute_name: str, value: float):
        usd_path = Sdf.Path(prim_path + "." + attribute_name)
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=usd_path,
            value=value,
            prev=self.get_property(prim_path, attribute_name),
        )

    def get_property(self, prim_path: str, attribute: str):
        prim = self.stage.GetPrimAtPath(prim_path)
        prim_property = prim.GetAttribute(attribute)
        return prim_property.Get()

    def create_payload(self, prim_path: str, asset_path: str):
        omni.kit.commands.execute('CreatePayload',
                                  usd_context=omni.usd.get_context(),
                                  path_to=Sdf.Path(prim_path),
                                  asset_path=asset_path,
                                  instanceable=False)

    def transform_prim(self, prim_path: str, x: float, y: float, z: float):
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
