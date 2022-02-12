
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
def generate_launch_description():

    slam_params_file = os.path.join(get_package_share_directory('owen_bringup'),
                                             'config', 'mapper_params_online_async.yaml')

    slam_config =  DeclareLaunchArgument(
        'slam_params_file',
        default_value = slam_params_file,
        description = 'launch config for mapper'
    )

    rplidar_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rplidar_ros2'), 'launch'),
         '/rplidar_launch.py'])
      )
    navigation_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('owen_bringup'), 'launch'),
         '/navigation.launch.py'])
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

    return LaunchDescription([
        slam_config,
        rplidar_launch,
        navigation_launch,
        create_launch,
        slam_launch,
        system_controller
    ])

