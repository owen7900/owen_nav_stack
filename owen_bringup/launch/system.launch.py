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

    args, argv = parser.parse_known_args()

    slam_params_file = os.path.join(get_package_share_directory('owen_bringup'),
                                             'config', 'mapper_params_online_async.yaml')

    slam_config =  DeclareLaunchArgument(
        'slam_params_file',
        default_value = slam_params_file,
        description = 'launch config for mapper'
    )

    if(not args.simulation):
        rplidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
             get_package_share_directory('rplidar_ros2'), 'launch'),
            '/rplidar_launch.py'])
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
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch'),
                    '/online_async_launch.py']),
        launch_arguments = {'slam_params_file': slam_params_file}.items()
        )

    system_controller = Node(
        package='system_controller',
        executable='system_controller_node',
        name='system_controller_node',
        output='screen'
    )

    ld = LaunchDescription([
        slam_config,
        navigation_launch,
        slam_launch,
        create_launch,
        system_controller
        ])

    if(args.simulation): 
        print("DOING SIM")
        ld.add_entity(simulation_launch)
    else:
        ld.add_entity(rplidar_launch)

    if(args.explore):
        print("EPLORING")
        ld.add_entity(explore_launch)


    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == "__main__":
    main()

