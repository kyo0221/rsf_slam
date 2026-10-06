import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    params_arg = DeclareLaunchArgument('params_file', default_value=config_file)
    georef_arg = DeclareLaunchArgument('georef', default_value='')
    georef = LaunchConfiguration('georef')
    use_gnss = PythonExpression(["'", georef, "' != ''"])
    params_file = LaunchConfiguration('params_file')
    bag = LaunchConfiguration('bag')
    output = LaunchConfiguration('output')
    rate = LaunchConfiguration('rate')
    start_offset = LaunchConfiguration('start_offset')

    scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[params_file],
        remappings=[('cloud_in', '/rsf/hokuyo_cloud2'), ('scan', '/scan')],
        output='screen',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'slam_params_file': params_file,
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
        condition=UnlessCondition(use_gnss),
    )

    gnss_bag_player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag, '--clock', '--rate', rate,
            '--start-offset', start_offset, '--topics',
            '/rsf/lio_lidar_rate_odom', '/rsf/hokuyo_cloud2',
        ],
        output='screen',
        condition=IfCondition(use_gnss),
    )

    ideal_switch_odom = Node(
        package='rsf_slam',
        executable='ideal_switch_odom.py',
        name='ideal_switch_odom',
        parameters=[params_file, {'bag': bag, 'georef': georef}],
        output='screen',
        condition=IfCondition(use_gnss),
    )

    save_map = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', output, '-t', '/map'],
        output='screen',
    )

    save_after_playback = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=player,
                on_exit=[
                    LogInfo(msg='Bag playback finished; waiting for SLAM map update.'),
                    TimerAction(period=10.0, actions=[save_map]),
                ],
            )
        )
        for player in (bag_player, gnss_bag_player)
    ]

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
        params_arg,
        georef_arg,
        scan_node,
        slam,
        ideal_switch_odom,
        *save_after_playback,
        shutdown_after_save,
        TimerAction(period=2.0, actions=[bag_player]),
        RegisterEventHandler(
            OnProcessIO(
                target_action=ideal_switch_odom,
                on_stderr=lambda event: [gnss_bag_player] if b'loaded' in event.text else None,
            )
        ),
    ])
