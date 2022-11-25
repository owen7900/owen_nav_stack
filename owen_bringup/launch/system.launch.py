#!/usr/bin/python3
import os
import sys
from argparse import ArgumentParser
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource


def main(argv = sys.argv[1:]):

    parser = ArgumentParser(description='Launch the owen_nav_stack system')
    parser.add_argument('-s', '--simulation', action='store_true', help='run the stack in simulation', default=False)
    parser.add_argument('-e', '--explore', action='store_true', help='run the stack in explore mode', default=False)
    parser.add_argument('-l', '--localization', action='store_true', help='run the stack in localization mode, default is mapping mode', default=False)

    args, argv = parser.parse_known_args()

    if(not args.localization):
        slam_params_file = os.path.join(get_package_share_directory('owen_bringup'),
                                    'config', 'mapper_params_online_async.yaml')
        slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch'),
                    '/online_async_launch.py']),
        launch_arguments = {'slam_params_file': slam_params_file}.items()
        )

    else:
        slam_launch = Node(package='owen_bringup', executable='map_switcher.py', output='screen', name='map_switcher')

    if(not args.simulation):
        lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='lidar_node',
        output='screen',
        parameters=[{
            'serial_port' : '/dev/lidar',
            'frame_id' : 'laser',
            'angle_compensate' : True
        }]
        )
    else:
        simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('owen_bringup'), 'launch', 'simulation.launch.py')])
            )

    navigation_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('owen_bringup'), 'launch'),
         '/navigation.launch.py'])
      )
    if(args.explore):
        explore_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('explore_lite'), 'launch', 'explore.launch.py')])
            )

    create_launch = IncludeLaunchDescription(
      AnyLaunchDescriptionSource([os.path.join(
         get_package_share_directory('create_bringup'), 'launch'),
         '/create_2.launch'])
      )
    system_controller = Node(
        package='system_controller',
        executable='system_controller_node',
        name='system_controller_node',
        output='screen'
    )

    ld = LaunchDescription([
        navigation_launch,
        slam_launch,
        system_controller
        ])

    if(args.simulation): 
        print("DOING SIM")
        ld.add_entity(simulation_launch)
    else:
        ld.add_action(lidar_node)
        ld.add_entity(create_launch)

    if(args.explore):
        print("EXPLORING")
        ld.add_entity(explore_launch)


    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == "__main__":
    main()

