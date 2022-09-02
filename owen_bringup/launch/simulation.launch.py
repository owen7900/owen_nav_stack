import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  urdf_file_name = 'urdf/create_2.urdf.xacro'


  urdf = os.path.join(
      get_package_share_directory('create_description'),
      urdf_file_name)

  world = os.path.join(get_package_share_directory('owen_bringup'), 'worlds', 'willowGarage.world')

  create_desc = get_package_share_directory('create_description')
  print("urdf_file_name : {}".format(urdf_file_name))
  return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH',value=create_desc+"/../"),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '--world', world],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                {'robot_description': xacro.process_file(urdf).toxml()}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "create_2"])
  ])
