#! /bin/python3
from threading import Thread, Lock
from launch import LaunchDescription, LaunchService
import rclpy
from rclpy import node
from launch_ros.actions import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import os

rclpy.init()
node = rclpy.create_node('map_switcher')

thread = Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()

l = Lock();

def floor_callback(msg):
    global need_to_restart, slam_launch, ld, ls, current_loaded_floor, l
    if msg.data != current_loaded_floor:
        ls.shutdown()
        current_loaded_floor = msg.data
        slam_params_file = os.path.join(get_package_share_directory('owen_bringup'), 'config', current_loaded_floor + '_mapper_params_localization.yaml')
        slam_launch = Node(
                    parameters=[
                        slam_params_file
                    ],
                    package='slam_toolbox',
                    executable='localization_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen'
                    )
        need_to_restart = True
        ld = LaunchDescription([slam_launch])
        l.acquire(blocking=True)
        ls = LaunchService()
        ls.include_launch_description(ld)
        l.release()
        
        print("RUN DONE")


floor_sub = node.create_subscription(String, '/floor_id', floor_callback, 1)
current_loaded_floor = ""
slam_params_file = "" 
ls = LaunchService()
ld = LaunchDescription()

rate = node.create_rate(1)

try:
    while rclpy.ok():
        l.acquire(blocking=True)
        ls.run()
        l.release()
        rate.sleep()
except KeyboardInterrupt:
    pass

rclpy.shutdown()
thread.join()
