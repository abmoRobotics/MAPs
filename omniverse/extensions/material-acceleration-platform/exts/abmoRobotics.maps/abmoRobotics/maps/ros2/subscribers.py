from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from ..package.planar_motor import PlanarMotor


class JointStateSubscriber(Node):
    def __init__(self, topic, prim_path: PlanarMotor):
        super().__init__("joint_state_subscriber")
        self.subscription = self.create_subscription(JointState, topic, self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.prim_path = prim_path
        self.position = 0

    def listener_callback(self, msg: JointState):
        if not self.position == msg.position:
            self.prim_path.set_joint_target(msg.position)
            self.position = msg.position


class InitialPoseSubscriber(Node):
    def __init__(self, topic, prim_path: PlanarMotor):
        super().__init__("initial_state_subscriber")
        self.subscription = self.create_subscription(Pose, topic, self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.prim_path = prim_path

    def listener_callback(self, msg: Pose):
        self.prim_path.set_initial_transform(msg.position.x * 100, msg.position.y * 100, msg.position.z * 100)
