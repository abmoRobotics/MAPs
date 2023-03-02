from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # True / False arguments 
    gui = LaunchConfiguration('gui')
    isaac_sim = LaunchConfiguration('isaac_sim')
    shuttle = LaunchConfiguration('sim_shuttle')
    manipulator = LaunchConfiguration('sim_manipulator')
    physical = LaunchConfiguration('use_physical_setup')
    no_shuttles = LaunchConfiguration('number_of_shuttles')

    # Decalre arguments
    gui_launch_arg = DeclareLaunchArgument(
        'gui1',
        default_value='False'
    )

    isaac_sim_launch_arg = DeclareLaunchArgument(
        'isaac_sim',
        default_value='True'
    )

    shuttle_launch_arg = DeclareLaunchArgument(
        'sim_shuttles',
        default_value='True'
    )

    manipulator_launch_arg = DeclareLaunchArgument(
        'sim_manipulator',
        default_value='True'
    )

    physical_launch_arg = DeclareLaunchArgument(
        'use_physical_setup',
        default_value='False'
    )
    num_of_shuttle_launch_arg = DeclareLaunchArgument(
        'number_of_shuttles',
        default_value="1"
    )
    shuttle_node = Node(
        package='orchestrator',
        namespace='shuttle',
        executable='shuttle',
        name='shuttle',
        parameters=[
            {"num_of_shuttles":no_shuttles}
        ]
    )
    manipulator_node = Node(
        package='orchestrator',
        namespace='shuttles1',
        executable='manipulator',
        name='sim'
    )
    task_planner_node = Node(
        package='orchestrator',
        namespace='shuttles2',
        executable='task_planner',
        name='sim'
    )

    return LaunchDescription([
        gui_launch_arg,
        isaac_sim_launch_arg,
        shuttle_launch_arg,
        manipulator_launch_arg,
        physical_launch_arg,
        num_of_shuttle_launch_arg,
        shuttle_node
       
    ])
