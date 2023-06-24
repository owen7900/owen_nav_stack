#!/usr/bin/python3
import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    simulation = LaunchConfiguration('simulation')
    localization = LaunchConfiguration('localization')
    use_nav_2 = LaunchConfiguration('use_nav_2')

    simulation_arg = DeclareLaunchArgument("simulation", default_value='True')
    localization_arg = DeclareLaunchArgument("localization", default_value='False')
    use_nav_2_arg = DeclareLaunchArgument('use_nav_2', default_value='False')

    slam_params_file = os.path.join(get_package_share_directory('owen_bringup'),
                                    'config', 'mapper_params_online_async.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch'),
                    '/online_async_launch.py']),
                    launch_arguments = {'slam_params_file': slam_params_file}.items(), 
                    condition=IfCondition(PythonExpression(['not ', localization]))
        )
    slam_params_file = os.path.join(get_package_share_directory('owen_bringup'), 'config',
                                        '1_mapper_params_localization.yaml')
    localization_launch = Node(
            parameters=[
                slam_params_file
            ],
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            condition=IfCondition(PythonExpression([localization]))
        )

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='lidar_node',
        #output='screen',
        parameters=[{
            'serial_port' : '/dev/lidar',
            'frame_id' : 'laser',
            'angle_compensate' : True
            }]
        )
    simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('owen_bringup'), 'launch', 'simulation.launch.py')]),
            condition=IfCondition(PythonExpression([simulation]))
            )


    navigation_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('owen_bringup'), 'launch'),
                                     '/navigation.launch.py']),
      condition=IfCondition(PythonExpression([use_nav_2])))

    custom_nav = Node(
            package='owen_navigation',
            executable='owen_navigation_node',
            name='owen_navigation_node',
            condition=IfCondition(PythonExpression(['not ', use_nav_2])),
            output='screen'
            )

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
        remappings=[('/roomba/cmd_vel', 
                     PythonExpression(['\'/diff_drive_base_controller/cmd_vel_unstamped\' if ', simulation, 
                                       ' else \'/roomba/cmd_vel\' ']))]
        #output='screen'
    )


    ld = LaunchDescription([
        simulation_arg,
        localization_arg,
        use_nav_2_arg,
        simulation_launch,
        slam_launch,
        system_controller,
        create_launch,
        navigation_launch,
        localization_launch,
        lidar_node,
        # custom_nav
        ])

    return ld;

if __name__ == "__main__":
    ls = LaunchService(argv=sys.argv)
    ls.include_launch_description(generate_launch_description())
    ls.run()


