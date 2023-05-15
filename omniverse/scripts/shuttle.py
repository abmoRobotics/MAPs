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
#from abmoRobotics.maps.package.six_axis import SixAxis
from abmoRobotics.maps.ros2 import JointStateSubscriber, InitialPoseSubscriber
import asyncio
import rclpy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from rclpy.node import Node
import numpy as np
import copy
from typing import List
import random
from omni.isaac.debug_draw import _debug_draw

def generate_random_goals(num_shuttles):
    desired_positions = np.random.rand(num_shuttles, 2)
    desired_positions[:, 0] = desired_positions[:, 0] * 0.96
    desired_positions[:, 1] = desired_positions[:, 1] * 0.72
    # Check if any of the desired positions are within a manhatten distance of 2 of each other
    invalid = np.sum(np.abs(desired_positions[:, None, :] - desired_positions[None, :, :]), axis=-1) < 0.2
    np.fill_diagonal(invalid, False)
    invalid = np.any(invalid)
    while invalid:
        desired_positions = np.random.rand(num_shuttles, 2)
        desired_positions[:, 0] = desired_positions[:, 0] * 0.96
        desired_positions[:, 1] = desired_positions[:, 1] * 0.72
        invalid = np.sum(np.abs(desired_positions[:, None, :] - desired_positions[None, :, :]), axis=-1) < 0.2
        np.fill_diagonal(invalid, False)
        invalid = np.any(invalid)

    return desired_positions


def setup(db):
    db.stage = omni.usd.get_context().get_stage()
    shuttle_prefix = "shuttle_120x120"
    remove_shuttle_paths = [x.GetPrimPath() for x in db.stage.Traverse() if (x.GetTypeName() == "Xform" and shuttle_prefix in str(x.GetPrimPath()))]
    print(remove_shuttle_paths)
    omni.kit.commands.execute('DeletePrims',
                              paths=remove_shuttle_paths,
                              destructive=True)

    if not rclpy.utilities.ok():
        rclpy.init()

    db.dummy_node = rclpy.create_node('dummy')

    db.shuttles = []
    db.joint_state_subscriber = []
    db.initial_pose_subscriber = []

    db.potentials = np.zeros((4,2))
    db.target_positions = np.zeros((4,6))
    goals = generate_random_goals(4)
    goals[:4,0:2] = [[0.9,0.6],[0.8,0.1],[0.1,0.6],[0.12,0.18]]
    db.target_positions[:, 0:2] = goals

    db.draw = _debug_draw.acquire_debug_draw_interface()
    N = 10000
    point_list_1 = [(0, 0, 0)]
    point_list_2 = [(1, 1, 1)]
    colors = [(0, 0, 255, 1)]
    sizes = [3]
    db.draw.draw_lines(point_list_1, point_list_2, colors, sizes)
    #draw.clear_lines()

debug = False
def compute(db):
    shuttle_topics = [x[0] for x in db.dummy_node.get_topic_names_and_types() if ("shuttle" in x[0] and x[0].endswith("joint_command") and "JointState" in x[1][0])]
    velocities = np.zeros((len(db.shuttles), 3))
    positions = np.zeros((len(db.shuttles), 3))
    target_positions = np.zeros((len(db.shuttles), 6))
    #print(db.target_positions)
    db.draw.clear_lines()
    for idx, shuttle in enumerate(db.shuttles):
        try:
            velocities[idx] = shuttle.get_joint_velocity()
            positions[idx] = shuttle.get_joint_position()
            target_positions[idx] = shuttle.get_joint_target()
            #print(db.target_positions[idx].shape)
            # print(f'db.target_positions.shape: {db.target_positions.shape}')
            # print(f'shuttle.get_joint_target.shape: {shuttle.get_joint_target().shape}')
            #print(f'target_positions: {target_positions}')
        except Exception as e:
            print(e)
            pass
    
    control_signal = np.zeros((len(db.shuttles), 2))
    db.potentials = db.potentials * 0.9
    a = 0
    for idx, shuttle in enumerate(db.shuttles):
        pass
        #shuttle.apply_control_input()current_points
    try:
        db.potentials, control_signal = db.shuttles[0].apply_force_field_controller(copy.deepcopy(positions), copy.deepcopy(velocities) , copy.deepcopy(target_positions), db.potentials)
    except Exception as e:
        print(e)
        #pass

    for idx, shuttle in enumerate(db.shuttles):
        #shuttle_input = [control_signal[idx, 0], control_signal[idx, 1],0,0,0,0]
        shuttle.set_target_velocity([control_signal[idx, 0], control_signal[idx, 1],0,0,0,0])


    ## Spawn new shuttles, and create subscribers for them if they are not already spawned.
    for idx, topic in enumerate(shuttle_topics):
        #print(idx)
        sdf_path = Sdf.Path(f"/World/tableScaled/joints/shuttle_120x120_{idx:02}")
        #print(sdf_path)
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
                                          paths=[Sdf.Path(f"/World/tableScaled/joints/shuttle_120x120_{idx:02}")],
                                          destructive=False)

    for joint_state_subscriber in db.joint_state_subscriber:
        rclpy.spin_once(joint_state_subscriber, timeout_sec=0.00000000000000001)
    for initial_pose_subscriber in db.initial_pose_subscriber:
        rclpy.spin_once(initial_pose_subscriber, timeout_sec=0.00000000000000001)


    try:
        targets_points = [(target_positions[i, 0]-1.2845, target_positions[i, 1]+0.3251, .99) for i in range(4)]
        current_points = [(positions[i, 0]-1.2845, positions[i, 1]+0.3251, 0.99) for i in range(4)]    
        colors = [(0, 255, 0, 1) for i in range(4)]
        sizes = [3 for i in range(4)]
        db.draw.draw_lines(targets_points, current_points, colors, sizes)
    except Exception as e:
        print(e)
        pass
    if debug:
        try:
            control_signal_points = [(positions[i, 0]-1.2845+control_signal[i, 0], positions[i, 1]+0.3251+control_signal[i, 1], 0.99) for i in range(4)]
            # #control_signal_points = [(db.target_positions[i, 0]-1.2845, db.target_positions[i, 1]+0.3251, 1) for i in range(5)]
            current_points = [(positions[i, 0]-1.2845, positions[i, 1]+0.3251, .99) for i in range(4)]       
            colors = [(0, 0, 255, 1) for i in range(4)]
            sizes = [10 for i in range(4)]
            db.draw.draw_lines(control_signal_points, current_points, colors, sizes)
        except Exception as e:
            # print(f'positions shape: {positions.shape}')
            # print(f'control_signal shape: {control_signal.shape}')
            #print(e)
            pass
    if debug:
        try:
            control_signal_points = [(positions[i, 0]-1.2845+db.potentials[i, 0], positions[i, 1]+0.3251+db.potentials[i, 1], 0.99) for i in range(4)]
            # #control_signal_points = [(db.target_positions[i, 0]-1.2845, db.target_positions[i, 1]+0.3251, 1) for i in range(4)]
            current_points = [(positions[i, 0]-1.2845, positions[i, 1]+0.3251, .99) for i in range(4)]       
            colors = [(255, 0, 0, 1) for i in range(4)]
            sizes = [4 for i in range(4)]
            db.draw.draw_lines(control_signal_points, current_points, colors, sizes)
        except Exception as e:
            # print(f'positions shape: {positions.shape}')
            # print(f'control_signal shape: {control_signal.shape}')
            #print(e)
            pass
            
def cleanup(db):
    print("SHUTTING DOWN")
    rclpy.shutdown()

    #ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "{name: ['<your-name>', '<your-second-name>'], position: [0.0, 0.4], velocity: [1.0, 1.2], effort: [0.0, 0.4]}"
    #ros2 topic pub -r 10 /shuttle/shuttle5/joint_command sensor_msgs/msg/JointState "{name: ['x','y','z','xrot','yrot','zrot'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"