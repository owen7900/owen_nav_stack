import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro
def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time', default=True)
  ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
  launch_dir = os.path.join(ros_gz_sim_dir, 'launch')

  # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  urdf_file_name = 'urdf/create_2.urdf.xacro'


  urdf = os.path.join(
      get_package_share_directory('create_description'),
      urdf_file_name)

  world = os.path.join(get_package_share_directory('owen_bringup'), 'worlds', 'willowGarage.world')

  create_desc = get_package_share_directory('create_description')
  print("urdf_file_name : {}".format(urdf_file_name))
  doc = xacro.parse(open(urdf))
  xacro.process_doc(doc)
  params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}
  pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

  spaw_robot = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-string', doc.toxml(),
                    '-name', 'create_2',
                    '-allow_renaming', 'true'],
                 output='screen')
  load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

  load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )
  bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
  return LaunchDescription([
        bridge,
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',value=[create_desc+"/../"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r  -v 4 empty.sdf'}.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spaw_robot,
                on_exit=[load_joint_state_controller]
                )
            ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller]
                )
            ),
        spaw_robot,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

  ])
