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
    shit_lidar = LaunchConfiguration('shit_lidar')

    simulation_arg = DeclareLaunchArgument("simulation", default_value='True')
    localization_arg = DeclareLaunchArgument("localization", default_value='False')
    shit_lidar_arg = DeclareLaunchArgument('shit_lidar', default_value='False')

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

    velodyne_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([
                                                os.path.join(get_package_share_directory('velodyne'), 
                                                'launch'), 
                                                '/velodyne-all-nodes-VLP16-composed-launch.py']), 
                                               condition=IfCondition(PythonExpression(['not ', shit_lidar, ' and not ', simulation]) )) 

    pointcloud_to_laserscan_node = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', ['/velodyne_points']),
                        ('scan', ['/scan'])],
            parameters=[{
                'target_frame': 'velodyne',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.007,  # M_PI/360.0
                'scan_time': 0.01,
                'range_min': 0.45,
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan',
            condition=IfCondition(PythonExpression(['not ', shit_lidar]))
        )

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
        condition=IfCondition(PythonExpression(['not ', simulation, ' and not ', shit_lidar]))
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
        shit_lidar_arg,
        map_features,
        pointcloud_to_laserscan_node,
        simulation_launch,
        slam_launch,
        system_controller,
        create_launch,
        navigation_launch,
        localization_launch,
        velodyne_launch,
        lidar_node,
#        elevator_traverser,
    #    apriltag_launch,
    #       master_navigator,
    #    apriltag_node
        ])

    return ld;

if __name__ == "__main__":
    ls = LaunchService(argv=sys.argv)
    ls.include_launch_description(generate_launch_description())
    ls.run()


