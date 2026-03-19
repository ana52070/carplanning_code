import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('campus_nav')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    urdf_path = os.path.join(pkg, 'urdf', 'turtlebot3_waffle.urdf')
    world_path = os.path.join(pkg, 'worlds', 'turtlebot3_world.world')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # 让 Gazebo 能找到我们自己的 model
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(pkg, 'models') + ':' +
                  os.path.join(pkg_tb3_gazebo, 'models')
        ),

        # 启动 Gazebo，加载我们自己的 world 文件
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

        # robot_state_publisher 加载我们自己的 URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
            }]
        ),

        # 在 Gazebo 中生成机器人
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
    ])
