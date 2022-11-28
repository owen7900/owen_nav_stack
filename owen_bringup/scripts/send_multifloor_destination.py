#! /bin/python3

import rclpy
from rclpy import node
from roomba_msgs.msg import MultifloorPoint
from argparse import ArgumentParser

rclpy.init()
node = rclpy.create_node(node_name='multifloor_destination_sender')

pub = node.create_publisher(MultifloorPoint, "/multifloor_destination", 1)

parser = ArgumentParser(description='multifloor_destination_sender')
parser.add_argument('-x', type=float, default=0.0)
parser.add_argument('-y', type=float, default=0.0)
parser.add_argument('-f', '--floor',  default="1")

args, argv = parser.parse_known_args()

msg = MultifloorPoint()

msg.floor_id.data = args.floor
msg.point.x = args.x
msg.point.y = args.y

pub.publish(msg)
