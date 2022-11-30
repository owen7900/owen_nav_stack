import rclpy
import time
from rclpy import node
from roomba_msgs.srv import GetAvailableDestinations
from std_msgs.msg import Empty

# Fake backend

rclpy.init()
node = rclpy.create_node('user_interface')
roomByFloor = {}
obstacleList = ["Ramp", "Elevator"]
rclpy.spin_once(node, timeout_sec=0.5)

destinationClient = node.create_client(GetAvailableDestinations, '/available_destinations')

print(node.get_service_names_and_types())

destinationClient.wait_for_service()

req = destinationClient.srv_type.Request()
availDest = destinationClient.call_async(req)

def callback(aDest):
    res = aDest.result()
    for dest in res.destinations:
        if not str(dest.floor_id.data) in roomByFloor:
            roomByFloor[str(dest.floor_id.data)] = []
        roomByFloor[dest.floor_id.data].append(dest.label.data)
    print(roomByFloor)

availDest.add_done_callback(callback)

i = 0

while not availDest.done() or i < 10:
    rclpy.spin_once(node, timeout_sec=0.1)
    i = i + 1

def getRoomsList():
    roomList = []
    for r in roomByFloor:
        roomList.append(roomByFloor[r])
    return roomList

def listByFloor(floor):
    return roomByFloor[str(floor)]

def getObstackeList():
    return obstacleList

