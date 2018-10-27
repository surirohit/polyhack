#!/usr/bin/env python
import requests

ENDPOINT = "http://10.4.14.37:5000/api"

SWARM_ID = "elcash"

CHANNEL = 80

RADIO = 0

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
    if drone_id is None:
        url = ENDPOINT + "/" + SWARM_ID + "/status"
        params = {}
    else:
        url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/status"
        params = {}
    r = requests.get(url=url, params=params)
    data = r.text
    print data
    
# Drone control functions

def stop(drone_id):
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/stop"
    params = {}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

def takeoff(drone_id, z, v):
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/takeoff"
    params = {"z": z, "v": v}
    r = requests.get(url=url, params=params)
    data = r.text
    print data
    
def land(drone_id, z, v):
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/land"
    params = {"z": z, "v": v}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

def goto(drone_id, x, y, z, yaw, v):
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/goto"
    params = {"x": x, "y": y, "z": z, "y": yaw, "v": v}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

# Packaging

def package():
    url = ENDPOINT + "/" + SWARM_ID + "/package"
    params = {}
    r = requests.get(url=url, params=params)
    data = r.text
    print data

def deliver(drone_id, package_id):
    url = ENDPOINT + "/" + SWARM_ID + "/" + drone_id + "/deliver"
    params = {"package": package_id}
    r = requests.get(url=url, params=params)
    data = r.text
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
    
if __name__ == '__main__':
    getArena()

