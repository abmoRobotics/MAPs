from ..base import Manipulator, SixAxisRobotInterace


class SixAxis(SixAxisRobotInterace):
    def __init__(self, manipulator_root: str, manipulator_index: int) -> None:
        super().__init__(manipulator_root, manipulator_index)
        self.manipulator_joint = self.manipulator_root + "/ActionGraph"

    def change_node_namespace(self, namespace: str):
        prim_path = self.manipulator_root + "/ActionGraph"
        attribute_name = 'inputs:nodeNamespace'
        self.omni_interface._change_property(prim_path=prim_path, attribute_name=attribute_name, value=namespace)

    def change_topic(self, topic: str):
        prim_path = self.manipulator_root + "/ActionGraph"
        attribute_name = 'inputs:topicName'
        self.omni_interface._change_property(prim_path=prim_path, attribute_name=attribute_name, value=topic)

    def spawn(self, asset_name: str = 'KR3R540'):
        prim_path = self.manipulator_root
        asset_path = f'./assets/robots/{asset_name}.usd'
        self.omni_interface._create_payload(prim_path=prim_path, asset_path=asset_path)
        self._set_initial_transform(x=20 * self.manipulator_index + 6.0, y=6.0, z=1.0)

    def _set_initial_transform(self, x, y, z):
        print(str(x) + " " + str(y) + " " + str(z))
        prim_path = f'{self.manipulator_root}/shuttle'
        print("prim path" + prim_path)
        self.omni_interface._transform_prim(prim_path=prim_path, x=x, y=y, z=z)
