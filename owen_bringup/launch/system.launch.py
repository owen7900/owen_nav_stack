#!/usr/bin/python3
import os
import sys
from argparse import ArgumentParser
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    # parser = ArgumentParser(description='Launch the owen_nav_stack system')
    # parser.add_argument('-s', '--simulation', action='store_true', help='run the stack in simulation', default=False)
    # parser.add_argument('-l', '--localization', action='store_true', help='run the stack in localization mode, default is mapping mode', default=False)
    # args, sys.argv = parser.parse_known_args()

    simulation = LaunchConfiguration('simulation')
    localization = LaunchConfiguration('localization')

    simulation_arg = DeclareLaunchArgument("simulation", default_value='True')
    localization_arg = DeclareLaunchArgument("localization", default_value='False')

    slam_params_file = os.path.join(get_package_share_directory('owen_bringup'),
                                    'config', 'mapper_params_online_async.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch'),
                    '/online_async_launch.py']),
                    launch_arguments = {'slam_params_file': slam_params_file}.items(), 
                    condition=IfCondition(PythonExpression(['not ', localization]))
        )

    localization_launch = Node(package='owen_bringup', executable='map_switcher.py', output='screen', name='map_switcher', condition=IfCondition(PythonExpression([localization])))

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='lidar_node',
        output='screen',
        parameters=[{
            'serial_port' : '/dev/lidar',
            'frame_id' : 'laser',
            'angle_compensate' : True
            }],
        condition=IfCondition(PythonExpression(['not ', simulation]))
        )
    simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('owen_bringup'), 'launch', 'simulation.launch.py')]),
            condition=IfCondition(PythonExpression([simulation]))
            )

    navigation_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('owen_bringup'), 'launch'),
         '/navigation.launch.py']))

    create_launch = IncludeLaunchDescription(
      AnyLaunchDescriptionSource([os.path.join(
         get_package_share_directory('create_bringup'), 'launch'),
         '/create_2.launch']),
        condition=IfCondition(PythonExpression(['not ', simulation]))
      )
    system_controller = Node(
        package='system_controller',
        executable='system_controller_node',
        name='system_controller_node',
        output='screen'
    )

    ld = LaunchDescription([
        simulation_arg,
        localization_arg,
        simulation_launch,
        slam_launch,
        system_controller,
        create_launch,
        navigation_launch,
        localization_launch,
        lidar_node
        ])

    return ld;


if __name__ == "__main__":
    ls = LaunchService(argv=sys.argv)
    ls.include_launch_description(generate_launch_description())
    ls.run()


