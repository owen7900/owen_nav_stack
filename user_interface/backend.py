import json

# Fake backend

def getRoomsList():
    # Temp until integration with Roomba
    with open("rooms.json","r") as f:
        data = json.load(f)
    return data;

def listByFloor(floor):
    data = getRoomsList()
    rooms = []
    for i in data["Rooms"]:
        if i["floor"] == floor:
            rooms.append(str(i["room"]))
    return rooms

def getObstackeList():
    with open("obstacles.json","r") as f:
        data = json.load(f)
    return data["Obstacles"];
