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
                    get_package_share_directory(package_name),'launch','online_async.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    nav2_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params_real.yaml')

    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'params_file': nav2_params_file}.items()
    )

    delayed_navigation = TimerAction(period=8.0, actions=[navigation])

    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'view_mogbe_real.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file]
    )

    delayed_rviz = TimerAction(period=10.0, actions=[rviz_node])

    return LaunchDescription([
        twist_mux,
        slam_toolbox,
        delayed_navigation,
        delayed_rviz,
    ])