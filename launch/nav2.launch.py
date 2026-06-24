
from launch import LaunchDescription

from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os



def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.expanduser(

        '~/carplaning/nav_ws/src/campus_nav/config/nav2_params.yaml')



    return LaunchDescription([

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

            ),

            launch_arguments={

                'use_sim_time': 'true',

                'params_file': nav2_params,

            }.items()

        )

    ])

