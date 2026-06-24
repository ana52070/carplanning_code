"""
=============================================================================
 EKF 诊断启动文件 — 一键启动所有诊断工具
=============================================================================

用法:
    ros2 launch carplanning_code ekf_diagnostic.launch.py

或者在终端中手动运行:
    # 快速检查 (一次性快照)
    ros2 run carplanning_code ekf_quick_check.sh   # 需要先 source install/setup.bash
    # 或直接:
    bash src/scripts/ekf_quick_check.sh --watch 3

    # Python 探针 (持续监控)
    python3 src/scripts/ekf_probe.py --rate 2.0

    # 单个话题频率检查
    ros2 topic hz /odom
    ros2 topic hz /gps/fix
    ros2 topic hz /odometry/gps
    ros2 topic hz /cmd_vel

    # TF 检查
    ros2 run tf2_ros tf2_echo map base_link
    ros2 run tf2_tools view_frames
=============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('carplanning_code')
    # ros2 launch 安装后 scripts 在 share/carplanning_code/scripts/
    script_dir = os.path.join(pkg_dir, 'scripts')
    probe_py = os.path.join(script_dir, 'ekf_probe.py')
    quick_sh = os.path.join(script_dir, 'ekf_quick_check.sh')

    # 如果不在安装路径, 尝试源码路径
    if not os.path.exists(probe_py):
        ws_src = os.path.join(
            os.path.dirname(os.path.dirname(pkg_dir)),  # .../carplanning_code/
            'src', 'scripts'
        )
        probe_py = os.path.join(ws_src, 'ekf_probe.py')
        quick_sh = os.path.join(ws_src, 'ekf_quick_check.sh')

    log_file = LaunchConfiguration('log_file', default='/tmp/ekf_probe.log')

    return LaunchDescription([

        DeclareLaunchArgument(
            'log_file', default_value='/tmp/ekf_probe.log',
            description='探针日志文件路径'
        ),

        LogInfo(msg="══════ EKF 诊断工具启动中... ══════"),

        # ══════════════════════════════════════════════════════════
        # 探针 1: Python 探针 (带延迟, 等系统就绪)
        # ══════════════════════════════════════════════════════════
        TimerAction(period=1.0, actions=[
            ExecuteProcess(
                cmd=['python3', probe_py, '--rate', '2.0',
                     '--log', log_file],
                name='ekf_probe',
                output='screen',
            ),
        ]),

        # ══════════════════════════════════════════════════════════
        # 探针 2: 关键话题频率监控
        # ══════════════════════════════════════════════════════════
        TimerAction(period=2.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'hz', '/odometry/gps', '--window', '20'],
                name='gps_odom_hz',
                output='screen',
            ),
        ]),

        # ══════════════════════════════════════════════════════════
        # 探针 3: TF 频率监控
        # ══════════════════════════════════════════════════════════
        TimerAction(period=2.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'hz', '/tf', '--window', '20'],
                name='tf_hz',
                output='screen',
            ),
        ]),

    ])
