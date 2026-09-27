import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('rsf_slam')
    config_file = os.path.join(package_dir, 'config', 'slam_params.yaml')
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')

    bag_arg = DeclareLaunchArgument('bag')
    output_arg = DeclareLaunchArgument('output', default_value='/tmp/rsf_slam_map')
    rate_arg = DeclareLaunchArgument('rate', default_value='1.0')
    start_offset_arg = DeclareLaunchArgument('start_offset', default_value='0')
    bag = LaunchConfiguration('bag')
    output = LaunchConfiguration('output')
    rate = LaunchConfiguration('rate')
    start_offset = LaunchConfiguration('start_offset')

    scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[config_file],
        remappings=[('cloud_in', '/rsf/hokuyo_cloud2'), ('scan', '/scan')],
        output='screen',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'slam_params_file': config_file,
            'use_sim_time': 'true',
        }.items(),
    )

    bag_player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag, '--clock', '--rate', rate,
            '--start-offset', start_offset, '--topics',
            '/tf', '/tf_static', '/rsf/hokuyo_cloud2',
        ],
        output='screen',
    )

    save_map = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', output, '-t', '/map'],
        output='screen',
    )

    save_after_playback = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player,
            on_exit=[
                LogInfo(msg='Bag playback finished; waiting for SLAM map update.'),
                TimerAction(period=5.0, actions=[save_map]),
            ],
        )
    )

    shutdown_after_save = RegisterEventHandler(
        OnProcessExit(
            target_action=save_map,
            on_exit=[EmitEvent(event=Shutdown(reason='Map saved.'))],
        )
    )

    return LaunchDescription([
        bag_arg,
        output_arg,
        rate_arg,
        start_offset_arg,
        scan_node,
        slam,
        save_after_playback,
        shutdown_after_save,
        TimerAction(period=2.0, actions=[bag_player]),
    ])
