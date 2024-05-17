import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():

    package_name='mogbe'

    # Declare launch arguments for x, y, z positions
    declare_x_position = DeclareLaunchArgument('x', default_value='0.0', description='Initial x position')
    declare_y_position = DeclareLaunchArgument('y', default_value='0.0', description='Initial y position')
    declare_z_position = DeclareLaunchArgument('z', default_value='0.5', description='Initial z position')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'mogbe_one',
                                   '-x', LaunchConfiguration('x'),
                                   '-y', LaunchConfiguration('y'),
                                   '-z', LaunchConfiguration('z')],
                        output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=join(get_package_share_directory(package_name), "models")),
        
        declare_x_position,
        declare_y_position,
        declare_z_position,

        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
    ])