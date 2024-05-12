import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    package_name='mogbe'

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','online_async_launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    delayed_slam_toolbox = TimerAction(period=5.0, actions=[slam_toolbox])

    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    delayed_navigation = TimerAction(period=10.0, actions=[navigation])

    rviz_node = ExecuteProcess(
        cmd=['rviz2'],
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        twist_mux,
        slam_toolbox,
        #delayed_slam_toolbox,
        navigation,
        #delayed_navigation,
    ])