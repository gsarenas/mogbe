import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_name='mogbe'

    declare_x_position = DeclareLaunchArgument('x', default_value='0.0', description='Initial x position')
    declare_y_position = DeclareLaunchArgument('y', default_value='0.0', description='Initial y position')
    declare_z_position = DeclareLaunchArgument('z', default_value='0.5', description='Initial z position')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')

    twist_mux = Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
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
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    slam_toolbox_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online.async.yaml')

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','online_async.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'slam_params_file': slam_toolbox_params_file}.items()
    )

    nav2_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params_sim.yaml')

    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'params_file': nav2_params_file}.items()
    )

    delayed_navigation = TimerAction(period=8.0, actions=[navigation])

    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'view_mogbe_sim.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file]
    )

    delayed_rviz = TimerAction(period=10.0, actions=[rviz_node])

    return LaunchDescription([
        AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=join(get_package_share_directory(package_name), 'models')),
        
        declare_x_position,
        declare_y_position,
        declare_z_position,

        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        slam_toolbox,
        delayed_navigation,
        delayed_rviz,
    ])