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
    slam_params_file = os.path.join(get_package_share_directory('owen_bringup'), 'config',
                                        '1_mapper_params_localization.yaml')
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch'),
                    '/online_async_launch.py']),
                    launch_arguments = {'slam_params_file': slam_params_file}.items(), 
                    condition=IfCondition(PythonExpression([localization]))
        )
 
#    localization_launch = Node(
#            parameters=[
#                slam_params_file
#            ],
#            package='slam_toolbox',
#            executable='localization_slam_toolbox_node',
#            name='slam_toolbox',
#            output='screen',
#            condition=IfCondition(PythonExpression([localization]))
#        )


#    localization_launch = Node(package='owen_bringup', executable='map_switcher.py', output='screen', name='map_switcher', condition=IfCondition(PythonExpression([localization])))

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

    map_features = Node(
            package='map_features',
            executable='map_features',
            name='map_features',
            output='screen'
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

    elevator_traverser = Node(
            package='elevator_traverser',
            executable='elevator_traverser',
            name='elevator_traverser',
            output='screen'
            )

    master_navigator = Node(
            package='master_navigator',
            executable='master_navigator',
            name='master_navigator',
            output='screen'
            )

   # apriltag_launch = IncludeLaunchDescription(
   #         PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('apriltag_ros'), 'launch', 'tag_36h11_all.launch.py')]), condition=IfCondition(PythonExpression(['not ', simulation]))
   #         )

   # apriltag_node = Node(
   #         package='apriltag_ros',
   #         executable='apriltag_node',
   #         condition=IfCondition(PythonExpression([simulation])),
   #         remappings=[('/image_rect', '/george_cam/image_raw'),
   #                     ('/camera_info','/george_cam/camera_info')]
   #         )



    ld = LaunchDescription([
        simulation_arg,
        localization_arg,
        map_features,
#        simulation_launch,
        slam_launch,
        system_controller,
        create_launch,
        navigation_launch,
        localization_launch,
        lidar_node,
#        elevator_traverser,
    #    apriltag_launch,
        master_navigator,
    #    apriltag_node
        ])

    return ld;

if __name__ == "__main__":
    ls = LaunchService(argv=sys.argv)
    ls.include_launch_description(generate_launch_description())
    ls.run()


