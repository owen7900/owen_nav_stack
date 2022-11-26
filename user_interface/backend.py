import json

def getRoomsList():
    # Temp until integration with Roomba
    with open("Rooms.json","r") as f:
        data = json.load(f)
    return data;

def listByFloor(floor):
    data = getRoomsList()
    rooms = []
    for i in data["Rooms"]:
        if i["floor"] == floor:
            rooms.append(str(i["room"]))
    return rooms
