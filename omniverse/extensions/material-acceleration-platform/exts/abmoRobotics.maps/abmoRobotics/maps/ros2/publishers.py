from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from ..package.planar_motor import PlanarMotor


class JointStatePublisher(Node):
    def __init__(self, topic):
        super().__init__("shuttle_joint_state_publisher")
        self.publisher = self.create_publisher(JointState, topic, 10)

    def publish_joint_state(self, msg: JointState):
        self.publisher.publish(msg)
