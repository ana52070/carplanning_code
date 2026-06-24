import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                             ExecuteProcess, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg         = get_package_share_directory('carplanning_code')
    pkg_nav2    = get_package_share_directory('nav2_bringup')
    pkg_fastlio = get_package_share_directory('fastlio2')
    nav2_yaml   = os.path.join(pkg, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg, 'bringup.rviz')

    # wait_for_tf 作为独立进程，退出后触发 Nav2
    wait_tf_proc = ExecuteProcess(
        cmd=['python3',
             os.path.join(pkg, 'scripts', 'wait_for_tf.py'),
             '--parent', 'odom',
             '--child', 'base_link',
             '--timeout', '60'],
        name='wait_for_tf',
        output='screen',
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_yaml,
        }.items()
    )

    return LaunchDescription([

        # ── 1. 底盘驱动 ──────────────────────────────
        Node(
            package='myslam_car_driver',
            executable='driver_node',
            name='driver_node',
            output='screen',
        ),

        # ── 2. 静态 TF：map = odom (identity) ────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            name='map_to_odom_tf',
        ),

        # ── 3. GPS 天线静态 TF ───────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '-0.14', '--y', '0.0', '--z', '0.12',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'gps'],
            name='gps_tf_publisher',
        ),

        # ── 4. LiDAR 坐标系静态 TF ──────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.14', '--y', '0', '--z', '0.26',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
            name='lidar_tf_publisher',
        ),

        # ── 5. Mid-360 雷达驱动（延迟2s）────────────
        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'mid360.launch.py')
                )
            ),
        ]),

        # ── 6. RTK GPS 驱动（延迟2s）────────────────
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

        # ── 7. FAST-LIO2 LiDAR 里程计（延迟4s）─────
        TimerAction(period=4.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_fastlio, 'launch', 'lio_launch.py')
                )
            ),
        ]),

        # ── 8. LIO 协方差注入 Relay（延迟5s）────────
        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=['python3',
                     os.path.join(pkg, 'scripts', 'lio_odom_relay.py')],
                name='lio_odom_relay',
                output='screen',
            ),
        ]),

        # ── 9. Livox CustomMsg -> PointCloud2 转换（延迟5s）──
        TimerAction(period=5.0, actions=[
            Node(
                package='carplanning_code',
                executable='livox_to_pc2',
                name='livox_to_pc2',
                output='screen',
            ),
        ]),

        # ── 10. 等 odom->base_link TF 就绪（延迟5s 后开始等）──
        TimerAction(period=5.0, actions=[wait_tf_proc]),

        # ── 11. Nav2：由 wait_for_tf 退出事件触发 ───
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_tf_proc,
                on_exit=[nav2_launch],
            )
        ),

        # ── 12. RViz2（延迟3s，等静态 TF 发布后再起）──
        TimerAction(period=3.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
            ),
        ]),
    ])