# This script is executed the first time the script node computes, or the next time it computes after this script
# is modified or the 'Reset' button is pressed.

# The following callback functions may be defined in this script:
#     setup(db): Called immediately after this script is executed
#     compute(db): Called every time the node computes (should always be defined)
#     cleanup(db): Called when the node is deleted or the reset button is pressed (if setup(db) was called before)

# Defining setup(db) and cleanup(db) is optional, but if compute(db) is not defined then this script node will run
# in legacy mode, where the entire script is executed on every compute and the callback functions above are ignored.

# Available variables:
#    db: og.Database The node interface, attributes are db.inputs.data, db.outputs.data.
#                    Use db.log_error, db.log_warning to report problems.
#                    Note that this is available outside of the callbacks only to support legacy mode.
#    og: The OmniGraph module

# Import statements, function/class definitions, and global variables may be placed outside of the callbacks.
# Variables may also be added to the db.internal_state state object.

# Example code snippet:
import omni.kit.commands
from pxr import Sdf, Usd
import omni.usd
from abmoRobotics.maps.package.planar_motor import PlanarMotor
from abmoRobotics.maps.ros2 import JointStateSubscriber, InitialPoseSubscriber
import asyncio
import rclpy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from rclpy.node import Node

async def my_task(joint_prim_paths):
    
    subscribers = []
    for idx, m in enumerate(joint_prim_paths):
        print(idx)
        topic = f"/shuttle/shuttle{idx}/joint_command"
        subscriber = JointStateSubscriber(topic, joint_prim_paths[idx])
        topics = [x[0] for x in subscriber.get_topic_names_and_types()]
        # print(topics)

        subscribers.append(subscriber)

    for x in range(100):
        for idx, subscriber in enumerate(subscribers):
            rclpy.spin_once(subscriber, timeout_sec=0.001)
            # joint_prim_paths[idx]
        # print("heej" + str(x))
        await asyncio.sleep(0.1)
# def check_if_topics_exits
def setup(db):
    db.stage = omni.usd.get_context().get_stage()
    shuttle_prefix = "shuttle_120x120"
    remove_shuttle_paths = [x.GetPrimPath() for x in db.stage.Traverse() if (x.GetTypeName() == "Xform" and shuttle_prefix in str(x.GetPrimPath()))]
    print(remove_shuttle_paths)
    omni.kit.commands.execute('DeletePrims',
                              paths=remove_shuttle_paths,
                              destructive=True)
    # for path in remove_shuttle_paths:
    #     print(path)

    if not rclpy.utilities.ok():
        rclpy.init()

    db.dummy_node = rclpy.create_node('dummy')

    # asyncio.ensure_future(my_task())
    db.shuttles = []
    db.joint_state_subscriber = []
    db.initial_pose_subscriber = []

def compute(db):
    shuttle_topics = [x[0] for x in db.dummy_node.get_topic_names_and_types() if ("shuttle" in x[0] and x[0].endswith("joint_command") and "JointState" in x[1][0])]
    # print(bool(db.dummy_node.get_publishers_info_by_topic('/shuttle/shuttle5/joint_command')))

    for idx, topic in enumerate(shuttle_topics):
        
        sdf_path = Sdf.Path(f"/World/tableScaled/joints/shuttle_120x120_{idx}")
        prim: Usd.Prim = db.stage.GetPrimAtPath(sdf_path)
        if bool(db.dummy_node.get_publishers_info_by_topic(topic)):
            if not prim.IsValid():
                planar_motor = PlanarMotor("/World/tableScaled/joints/shuttle_120x120", idx)
                planar_motor.spawn()
                joint_state_subscriber = JointStateSubscriber(topic, planar_motor)
                initial_pose_subscriber = InitialPoseSubscriber(f"configuration/shuttle{idx}/initialPosition", planar_motor)
                db.shuttles.append(planar_motor)
                db.joint_state_subscriber.append(joint_state_subscriber)
                db.initial_pose_subscriber.append(initial_pose_subscriber)
        else:
            
            if prim.IsValid():
                omni.kit.commands.execute('DeletePrims',
                                          paths=[Sdf.Path(f"/World/tableScaled/joints/shuttle_120x120_{idx}")],
                                          destructive=False)

    for joint_state_subscriber in db.joint_state_subscriber:
        rclpy.spin_once(joint_state_subscriber, timeout_sec=0.00000000000000001)
    for initial_pose_subscriber in db.initial_pose_subscriber:
        rclpy.spin_once(initial_pose_subscriber, timeout_sec=0.00000000000000001)

def cleanup(db):
    print("SHUTTING DOWN")
    rclpy.shutdown()

    #ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "{name: ['<your-name>', '<your-second-name>'], position: [0.0, 0.4], velocity: [1.0, 1.2], effort: [0.0, 0.4]}"
    #ros2 topic pub -r 10 /shuttle/shuttle5/joint_command sensor_msgs/msg/JointState "{name: ['x','y','z','xrot','yrot','zrot'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"