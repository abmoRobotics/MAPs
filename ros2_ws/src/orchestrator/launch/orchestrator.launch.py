from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from utils import save_yaml
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)


def generate_launch_description():
    # True / False arguments 
    gui = LaunchConfiguration('gui')
    isaac_sim = LaunchConfiguration('isaac_sim')
    shuttle = LaunchConfiguration('sim_shuttle')
    manipulator = LaunchConfiguration('sim_manipulator')
    physical = LaunchConfiguration('use_physical_setup')
    no_shuttles = LaunchConfiguration('number_of_shuttles')

    config = os.path.join(
      get_package_share_directory('orchestrator'),
      'config',
      'initial_position.yaml'
      )

    # Decalre arguments
    gui_launch_arg = DeclareLaunchArgument(
        'gui',
        description="If you want a GUI with statistiscs and feedback",
        default_value='False'
    )

    isaac_sim_launch_arg = DeclareLaunchArgument(
        'isaac_sim',
        description="If you want the simuleated environment therefore also no shuttles or manipulators",
        default_value='True'
    )

    shuttle_launch_arg = DeclareLaunchArgument(
        'sim_shuttle',
        description="If you want the simulated environment with the shuttles",
        default_value='True'
    )

    manipulator_launch_arg = DeclareLaunchArgument(
        'sim_manipulator',
        description="If you want the simulated environment with the manipulators",
        default_value='True'
    )

    physical_launch_arg = DeclareLaunchArgument(
        'use_physical_setup',
        description="If you want a physical setup or not",
        default_value='False'
    )
    num_of_shuttle_launch_arg = DeclareLaunchArgument(
        'number_of_shuttles',
        description="This will decalare the number of shuttles you want on the tabel",
        default_value="5"
    )
    shuttle_node = Node(
        package='orchestrator',
        namespace='shuttle',
        executable='shuttle',
        name='shuttle',
        output="screen",
        emulate_tty=True,
        parameters=[
            {"num_of_shuttles":no_shuttles},
            {"sim_shuttle":shuttle}
        ]
    )
    manipulator_node = Node(
        package='orchestrator',
        namespace='manipulator',
        executable='manipulator',
        name='sim'
    )
    task_planner_node = Node(
        package='orchestrator',
        namespace='task_planner',
        executable='task_planner',
        name='sim'
    )

    gui_node = Node(
        package='orchestrator',
        executable='gui',
        name='gui',
        output="screen",
        emulate_tty=True,
        parameters=[config,
                    {"num_of_shuttles":no_shuttles}]

    )


    return LaunchDescription([
        gui_launch_arg,
        isaac_sim_launch_arg,
        shuttle_launch_arg,
        manipulator_launch_arg,
        physical_launch_arg,
        num_of_shuttle_launch_arg,
        shuttle_node,
        gui_node,

        # RegisterEventHandler(
        #     OnShutdown(
        #         on_shutdown=[ExecuteProcess(save_yaml(shuttle_node)
        #         )]
        #     )
        # ),
       
    ])
