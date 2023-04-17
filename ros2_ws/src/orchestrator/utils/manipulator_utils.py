import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np
from PIL import Image


class KR3R540(DHRobot):
    """
    Class that models a KR 3 R540 manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

    ``KR3R540()`` is an object which models a KUKA KR 3 R540 robot and
    describes its kinematic standard DH conventions.

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - home, vertical 'READY' configuration
    """  # noqa

    def __init__(self, symbolic=False):
        if symbolic:
            import spatialmath.base.symbolic as sym
            pi = sym.pi()
        else:
            from math import pi

        super().__init__(
                [
                    RevoluteDH(a=-0.020 , alpha=-pi/2, d=-0.345 , offset=pi/2),
                    RevoluteDH(a=0.260),
                    RevoluteDH(a=0.020  , alpha=pi/2            , offset=-pi/2),
                    RevoluteDH(           alpha=-pi/2, d=-0.260),
                    RevoluteDH(           alpha=pi/2),
                    RevoluteDH(                        d=-0.075)
                ],  name = "KR 3 R540",
                    manufacturer = "KUKA",
                    keywords=("dynamics", "symbolic", "mesh"),
                    symbolic=symbolic,
                    meshdir="/home/rossen/Desktop/MAPs/ros2_ws/src/kuka_kr3_support/meshes/kr3r540/collision/"
                        )
        self.home = np.array([pi/4, pi / 2, pi / 2, pi/4, pi/4, pi/4])
        self.qz = np.zeros(6)
        
        self.addconfiguration("home", self.home)
        self.addconfiguration("zero", self.qz)


class KR4R600(DHRobot):
    """
    Class that models a KR 4 R600 manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

    ``KR4R600()`` is an object which models a KUKA KR 4 R600 robot and
    describes its kinematic standard DH conventions.

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - home, vertical 'READY' configuration
    """  # noqa

    def __init__(self, symbolic=False):
        if symbolic:
            import spatialmath.base.symbolic as sym
            pi = sym.pi()
        else:
            from math import pi

        super().__init__(
                [
                    RevoluteDH(alpha=-pi/2, d=-0.330 , offset=pi/2),
                    RevoluteDH(a=0.290),
                    RevoluteDH(a=0.020  , alpha=pi/2            , offset=-pi/2),
                    RevoluteDH(           alpha=-pi/2, d=-0.310),
                    RevoluteDH(           alpha=pi/2),
                    RevoluteDH(                        d=-0.075)
                ],  name = "KR 4 R600",
                    manufacturer = "KUKA",
                    keywords=("dynamics", "symbolic", "mesh"),
                    symbolic=symbolic,
                    meshdir="/home/rossen/Desktop/MAPs/ros2_ws/src/kuka_kr3_support/meshes/kr3r540/collision/"
                        )
        self.home = np.array([pi/4, pi / 2, pi / 2, pi/4, pi/4, pi/4])
        self.qz = np.zeros(6)
        
        self.addconfiguration("home", self.home)
        self.addconfiguration("zero", self.qz)