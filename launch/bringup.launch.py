import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('campus_nav')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    urdf_path  = os.path.join(pkg, 'urdf', 'turtlebot3_waffle.urdf')
    world_path = os.path.join(pkg, 'worlds', 'turtlebot3_world.world')
    ekf_yaml   = os.path.join(pkg, 'config', 'ekf.yaml')
    navsat_yaml= os.path.join(pkg, 'config', 'navsat.yaml')
    nav2_yaml  = os.path.join(pkg, 'config', 'nav2_params.yaml')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([

        # ===== 仿真环境 =====
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(pkg, 'models') + ':' +
                  os.path.join(pkg_tb3_gazebo, 'models')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'waffle',
                '-file', os.path.join(pkg, 'models', 'turtlebot3_waffle', 'model.sdf'),
                '-x', '-2.0', '-y', '-0.5', '-z', '0.01'
            ],
            output='screen'
        ),

        # ===== EKF 定位（延迟3秒等Gazebo就绪）=====
        TimerAction(period=3.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                parameters=[ekf_yaml, {'use_sim_time': True}],
                output='screen'
            ),
        ]),

        # ===== navsat_transform（延迟6秒等EKF就绪）=====
        TimerAction(period=6.0, actions=[
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform',
                parameters=[navsat_yaml, {'use_sim_time': True}],
                output='screen'
            ),
        ]),

        # ===== Nav2（延迟8秒）=====
        TimerAction(period=8.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_yaml,
                }.items()
            ),
        ]),

        # ===== Rviz（延迟10秒）=====
        TimerAction(period=10.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'rviz_launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            ),
        ]),
    ])
