import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg         = get_package_share_directory('carplanning_code')
    pkg_nav2    = get_package_share_directory('nav2_bringup')

    ekf_yaml    = os.path.join(pkg, 'config', 'ekf.yaml')
    navsat_yaml = os.path.join(pkg, 'config', 'navsat.yaml')
    nav2_yaml   = os.path.join(pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # ── 1. 底盘驱动 ──────────────────────────────
        Node(
            package='myslam_car_driver',
            executable='driver_node',
            name='driver_node',
            output='screen',
        ),

        # ── 2. Mid-360 雷达驱动（延迟2s）────────────
        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('livox_ros_driver2'),
                        'launch_ROS2', 'msg_MID360s_launch.py'
                    )
                )
            ),
        ]),

        # ── 3. RTK 驱动（延迟2s）────────────────────
        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('um982_ros2_driver'),
                        'launch', 'um982_ros2_driver.launch.py'
                    )
                )
            ),
        ]),

        # ── 4. EKF 定位（延迟5s）────────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                parameters=[ekf_yaml],
                output='screen',
            ),
        ]),

        # ── 5. navsat_transform（延迟8s）────────────
        TimerAction(period=8.0, actions=[
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform',
                parameters=[navsat_yaml],
                output='screen',
            ),
        ]),

        # ── 6. Nav2（延迟10s）───────────────────────
        TimerAction(period=10.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_yaml,
                }.items()
            ),
        ]),
    ])
