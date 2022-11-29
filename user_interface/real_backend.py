import rclpy
from roomba_msgs.srv import GetAvailableDestinations
from std_msgs.msg import Empty

# Fake backend

node = rclpy.Node('user_interface')
roomByFloor = {}
obstacleList = ["Ramp", "Elevator"]

destinationClient = node.create_client(GetAvailableDestinations, 'available_destinations')

availDest = destinationClient.call(Empty())
for dest in availDest.destinations:
    roomByFloor[dest.floor_id.data].append(dest.label.data)

def getRoomsList():
    roomList = []
    for r in roomByFloor:
        roomList.append(roomByFloor[r])
    return roomList

def listByFloor(floor):
    return roomByFloor[floor]

def getObstackeList():
    return obstacleList

rclpy.spin(node)
