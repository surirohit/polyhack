#!/usr/bin/env python
import requests
import rospy
from elcash_drone.msg import DroneStatus, DroneLandTakeoff, DroneCommand
from elcash_drone.msg import DronePackage

ENDPOINT = "http://10.4.14.37:5000/api"
SWARM_ID = "elcash"
CHANNEL = 80
RADIO = 0

drone = ['drone_11','drone_12','drone_13']
drone_id = ['E7E7E7E712','E7E7E7E711','E7E7E7E713']

N = 1

def getArena():
    url = ENDPOINT + "/arena"
    params = {}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

# Swarm setup function

def registerSwarm(arena, seed):
    url = ENDPOINT + "/" + SWARM_ID + "/register_swarm"
    params = {"arena_id": 0, "seed": seed}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

# Drone setup functions

def connect(drone_id, drone_address):
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/connect"
    params = {"r": RADIO, "c": CHANNEL, "dr": "2M", "a": drone_address}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

def disconnect(drone_id):
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/disconnect"
    params = {}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

def calibrate(drone_id):
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/calibrate"
    params = {}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

# Status functions

def status(drone_id = None):
    global status_msg
    if drone_id is None:
        url = ENDPOINT + "/" + SWARM_ID + "/status"
        params = {}
    else:
        url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/status"
        params = {}
    r = requests.get(url=url, params=params)
    try:
        #print r.text
        data = r.json()
        if drone_id is None:
            return False

        status_msg.id = data["id"]
        status_msg.var_x = data["var_x"]
        status_msg.var_y = data["var_y"]
        status_msg.var_z = data["var_z"]
        status_msg.x = data["x"]
        status_msg.y = data["y"]
        status_msg.z = data["z"]
        status_msg.yaw = data["yaw"]
        status_msg.status = data["status"]
        status_msg.battery_voltage = data["battery_voltage"]
        status_msg.battery_percentage = data["battery_percentage:"]

        return True
    except ValueError:
        return False

# Drone control functions

def stop(drone_id):
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/stop"
    params = {}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

def takeoff(takeoff_msg):
    drone_id = takeoff_msg.drone_id
    z = takeoff_msg.height
    v = takeoff_msg.velocity
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/takeoff"
    params = {"z": z, "v": v}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

def land(land_msg):
    drone_id = land_msg.drone_id
    z = land_msg.height
    v = land_msg.velocity
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/land"
    params = {"z": z, "v": v}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

def goto(cmd_msg):
    drone_id = cmd_msg.drone_id
    x = cmd_msg.x
    y = cmd_msg.y
    z = cmd_msg.z
    yaw = cmd_msg.yaw
    v = cmd_msg.velocity

    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/goto"
    params = {"x": x, "y": y, "z": z, "yaw": yaw, "v": v}

    r = requests.get(url=url, params=params)
    data = r.text
    print data

# Packaging

def package():
    print "Trying to get package"
    global need_to_assign,package_msg,package_pub,drone
    for i in range(len(need_to_assign)):
        if need_to_assign[i]:
            url = ENDPOINT + "/" + SWARM_ID + "/package"
            params = {}
            r = requests.get(url=url, params=params)
            try:
                #print r.text
                data = r.json()
                package_msg.id = data["id"]
                package_msg.x = data["coordinates"][0]
                package_msg.y = data["coordinates"][1]
                package_msg.weight = data["weight"]
                package_msg.drone_id = drone[i]
                package_pub[i].publish(package_msg)
                print package_msg.id
            except ValueError:
                return
            need_to_assign[i] = False

def deliver(delivered_msg):
    global need_to_assign
    drone_id = delivered_msg.drone_id
    package_id = delivered_msg.id
    for i in range(N):
        if drone_id == drone[i]:
            need_to_assign[i] = True

    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/deliver"
    params = {"package": package_id}
    r = requests.get(url=url, params=params)
    data = r.text
    print "CHECK THIS!!!!!!!!!!!!!!!!!!!!!!!!!!"
    print data

# Misc

def resetGenerator(seed):
    url = ENDPOINT + "/" + SWARM_ID + "/reset_package_generator"
    params = {"seed": seed}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

def print_deliveries():
    url = ENDPOINT + "/" + SWARM_ID + "/print_deliveries"
    params = {}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

def shutdown():
    url = ENDPOINT + "/" + SWARM_ID + "/shutdown"
    params = {}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

status_msg = DroneStatus()
package_msg = DronePackage()

need_to_assign = [True, False, False]

rospy.init_node("server", anonymous=False)

status_pub = []
package_pub = []
goto_sub = []
land_sub = []
takeoff_sub = []

## Dont need different Subscribers because drone_id is in package
rospy.Subscriber('/delivered', DronePackage, deliver)

for i in range(N):
    status_pub.append(rospy.Publisher('/'+drone[i]+'/status', DroneStatus, queue_size=1))
    package_pub.append(rospy.Publisher('/'+drone[i]+'/assign', DronePackage, queue_size=1))

    goto_sub.append(rospy.Subscriber('/'+drone[i]+'/goto', DroneCommand, goto))
    land_sub.append(rospy.Subscriber('/'+drone[i]+'/land', DroneLandTakeoff, land))
    takeoff_sub.append(rospy.Subscriber('/'+drone[i]+'/takeoff', DroneLandTakeoff, takeoff))

rate = rospy.Rate(2)

registerSwarm(0,1234)
for i in range(N):
    connect(drone[i],drone_id[i])
    calibrate(drone[i])

while not rospy.is_shutdown():
    for i in range(N):
        if status(drone_id=drone[i]):
            status_pub[i].publish(status_msg)

    if True in need_to_assign:
        package()
    rate.sleep()

for i in range(N):
    disconnect(drone[i])
