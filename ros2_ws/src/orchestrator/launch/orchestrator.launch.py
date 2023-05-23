from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from utils import save_yaml
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)


def generate_launch_description():
    # True / False arguments 
    gui = LaunchConfiguration('gui')
    isaac_sim = LaunchConfiguration('isaac_sim')
    shuttle = LaunchConfiguration('sim_shuttle')
    no_shuttles = LaunchConfiguration('num_of_shuttles')
    manipulator = LaunchConfiguration('sim_manipulator')
    no_manipulators = LaunchConfiguration('num_of_manipulators')
    no_table = LaunchConfiguration('number_of_tabels')
    manipulator_position = LaunchConfiguration('manipulator_position')
    table_position = LaunchConfiguration('table_position')

    global_parameters = os.path.join(
        get_package_share_directory('acopos_bringup'),
        'config',
        'global_params.yaml'
    )
    

    config = os.path.join(
      get_package_share_directory('orchestrator'),
      'config',
      'initial_position.yaml'
      )

    # Decalre arguments
    gui_launch_arg = DeclareLaunchArgument(
        'gui',
        description="If you want a GUI with statistiscs and feedback.",
        default_value='False',
        choices=['True', 'False']
    )
    isaac_sim_launch_arg = DeclareLaunchArgument(
        'isaac_sim',
        description="If you want the simuleated environment therefore also no shuttles or manipulators.",
        default_value='True',
        choices=['True', 'False']
    )
    shuttle_launch_arg = DeclareLaunchArgument(
        'sim_shuttle',
        description="If you want the simulated environment with the shuttles.",
        default_value='True',
        choices=['True', 'False']
    )
    manipulator_launch_arg = DeclareLaunchArgument(
        'sim_manipulator',
        description="If you want the simulated environment with the manipulators.",
        default_value='True',
        choices=['True', 'False']
    )
    physical_launch_arg = DeclareLaunchArgument(
        'use_physical_setup',
        description="If you want a physical setup or not.",
        default_value='False',
        choices=['True', 'False']
    )
    num_of_shuttle_launch_arg = DeclareLaunchArgument(
        'num_of_shuttles',
        description="This will decalare the number of shuttles you want on the tabel.",
        default_value="5"
    )
    
    num_of_manipulators_launch_arg = DeclareLaunchArgument(
        'num_of_manipulators',
        description="This will decalare the number of manipulators you want to the tabel.",
        default_value="6"
    )

    num_of_tabels_launch_arg = DeclareLaunchArgument(
        'num_of_tabels',
        description="This will decalare the number of tabels you want.",
        default_value="6"
    )

    manipulator_position_launch_arg = DeclareLaunchArgument(
        'manipulator_position',
        description="This will decalare the nmanipulators position.",
        default_value="5"
    )

    table_position_launch_arg = DeclareLaunchArgument(
        'table_position',
        description="This will decalare the number of tabels you want.",
        default_value="6"
    )

    num_of_tabels_launch_arg = DeclareLaunchArgument(
        'number_of_tabels',
        description="This will decalare the number of tabels you want.",
        default_value="6"
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
        name='manipulator',
        output="screen",
        emulate_tty=True,
        parameters=[config,
                    global_parameters,
                    {"num_of_manipulators":no_manipulators},
                    {"sim_manipulators":manipulator}
                    ]
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
                    global_parameters,
                    {"num_of_shuttles":no_shuttles},
                    {"num_of_manipulators":no_manipulators},
                    {"num_of_tabels":no_table},
                    {"table_position":table_position},
                    {"robot_position":manipulator_position} 
                    ]
    )

    spawn_node = Node(
        package='orchestrator',
        executable='spawn_manager',
        name='spawn_manager',
        output="screen",
        emulate_tty=True,
        parameters=[])


    return LaunchDescription([
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                    FindPackageShare("acopos_bringup"), '/launch', '/acopos.launch.py'])
        ),
        gui_launch_arg,
        isaac_sim_launch_arg,
        shuttle_launch_arg,
        manipulator_launch_arg,
        num_of_shuttle_launch_arg,
        num_of_manipulators_launch_arg,
        manipulator_position_launch_arg,
        num_of_tabels_launch_arg,
        table_position_launch_arg,
        spawn_node,
        gui_node,
        shuttle_node,
        manipulator_node,
        task_planner_node

        # RegisterEventHandler(
        #     OnShutdown(
        #         on_shutdown=[ExecuteProcess(save_yaml(shuttle_node)
        #         )]
        #     )
        # ),
       
    ])
