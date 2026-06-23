import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg         = get_package_share_directory('carplanning_code')
    pkg_nav2    = get_package_share_directory('nav2_bringup')
    pkg_fastlio = get_package_share_directory('fastlio2')

    ekf_yaml    = os.path.join(pkg, 'config', 'ekf.yaml')
    navsat_yaml = os.path.join(pkg, 'config', 'navsat.yaml')
    nav2_yaml   = os.path.join(pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # ── 1. 底盘驱动 ──────────────────────────────
        # 仍然需要: 提供 /cmd_vel 接收、/odom (备用)、TF odom→base_footprint
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
                    os.path.join(pkg, 'launch', 'mid360.launch.py')
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

        # ── 4. FAST-LIO2 LiDAR 里程计（延迟4s）─────
        # 等 LiDAR + IMU 数据稳定后再启动
        # 注意: 不启动 RViz (去掉 lio_launch.py 中的 rviz 或用下面的方式单独起节点)
        TimerAction(period=4.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_fastlio, 'launch', 'lio_launch.py')
                )
            ),
        ]),

        # ── 5. LIO 协方差注入 Relay（延迟5s）────────
        # FAST-LIO2 输出协方差全零, 注入合理值后 EKF 才能正常融合
        # 替代 v2 的 covariance_relay.py (轮式里程计+IMU)
        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=['python3',
                     os.path.join(pkg, 'scripts', 'lio_odom_relay.py')],
                name='lio_odom_relay',
                output='screen',
            ),
        ]),

        # ── 6. EKF 定位（延迟7s）────────────────────
        # 等 FAST-LIO2 + relay 稳定输出后再启动
        TimerAction(period=7.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                parameters=[ekf_yaml],
                output='screen',
            ),
        ]),

        # ── 7. navsat_transform（延迟10s）───────────
        TimerAction(period=10.0, actions=[
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform',
                parameters=[navsat_yaml],
                output='screen',
            ),
        ]),

        # ── 8. Nav2（延迟12s）───────────────────────
        TimerAction(period=12.0, actions=[
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

        # ── 9. GPS 天线静态变换 ──────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '-0.14', '--y', '0.0', '--z', '0.12',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'gps'],
            name='gps_tf_publisher',
        ),

        # ── 10. LiDAR 坐标系静态变换 ────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.14', '--y', '0', '--z', '0.26',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
            name='lidar_tf_publisher',
        ),
    ])
