#!/usr/bin/env python
import rospy
from elcash_drone.msg import DroneStatus, DroneLandTakeoff, DroneCommand
from elcash_drone.msg import DronePackage
import networkx as nx

LAND_TIME = 3

NODES = {
    "0": (2.2, 1.6),
    "2": (2.6, 0.6),
    "3": (3.4, 1.4),
    "4": (2.4, 3.4),
    "5": (0.6, 2.2),
    "6": (1.4, 3.2),
    "7": (1, 1.6),
    "8": (3.6, 0.6),
    "9": (3.2, 3.2),
    "I1": (2.6, 2.6),
    "I2": (3.6, 2.4),
    "I3" : (1.6, 2),
    "I4": (0.5, 0.5),
    "I5": (0.6, 3.4),
    "I6": (2.8, 1.2),
    "I7": (2, 1),
    "I8": (3.2, 0.2),
    "I9": (1.6, 0.5)

}
EDGES = [
    ("0", "2"),
    ("0", "3"),
    ("0", "I1"),
    ("0", "I3"),
    ("0", "7"),
    ("I4", "7"),
    ("7", "I3"),
    ("7", "5"),
    ("5", "I3"),
    ("5", "6"),
    ("5", "I5"),
    ("I5", "6"),
    ("6", "I3"),
    ("6", "4"),
    ("4", "I1"),
    ("I1", "9"),
    ("I1", "I3"),
    ("I1", "I2"),
    ("I2", "9"),
    ("I2", "3"),
    ("3", "8"),
    ("4", "9"),
    ("I6", "3"),
    ("I7", "2"),
    ("I7", "0"),
    ("I8", "2"),
    ("I8", "8"),
    ("5", "I4"),
    ("I4", "I9"),
    ("I9", "2"),
    ("I6", "0")
]

def init_graph():
    G = nx.Graph()
    weighted_edges = []
    for (a, b) in EDGES:
        dist = distance(NODES[a], NODES[b])
        weighted_edges.append((a, b, {'w': dist}))
        G.add_edges_from(weighted_edges);
    return G

def path(origin, target):
    G = init_graph()
    path_nodes = nx.shortest_path(G, origin, target, "w")
    path = []
    for node in path_nodes:
        path.append(NODES[node])
    return path

def best_match(position, nodes):
    min = float("inf")
    k_m = None
    for (node, pos) in nodes.items():
        dist = distance(position, pos)
        if dist < min:
            k_m = node
            min = dist
    return k_m

def plan(origin_pos, goal_pos):
    origin = best_match(origin_pos, NODES)
    target = best_match(goal_pos, NODES)
    return path(origin, target)

package_list = []
package_status = []

THRESH = 0.07

curr = (0,0)
home = (2.2,1.6) #TODO

def distance(pt1,pt2):
    return ((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)**0.5

def assign_package(package_msg):
    global package_list,package_status
    print "Got package",package_msg.id
    package_list.append(package_msg)
    package_status.append(False)

def update_status(status_msg):
    global curr
    curr = (status_msg.x,status_msg.y)

def goToPath(path):
    global curr,cmd_msg,THRESH
    for i in range(len(path)):
        pt = path[i]
        print "Going to",pt
        cmd_msg.x = pt[0]
        cmd_msg.y = pt[1]
        if i == len(path)-1:
            cmd_msg.z = cmd_msg.z/2
        cmd_pub.publish(cmd_msg)
        if i == len(path)-1:
            cmd_msg.z = cmd_msg.z*2
        print "Waiting"
        while distance(pt,curr) > THRESH:
            rospy.sleep(0.1)

drone_id = rospy.get_param('~drone_id', 'drone_11')
rospy.init_node(drone_id+"_commander", anonymous=False)

cmd_pub = rospy.Publisher(drone_id+'/goto', DroneCommand, queue_size=1)
land_pub = rospy.Publisher(drone_id+'/land', DroneLandTakeoff, queue_size=1)
takeoff_pub = rospy.Publisher(drone_id+'/takeoff', DroneLandTakeoff, queue_size=1)
delivered_pub = rospy.Publisher('/delivered', DronePackage, queue_size=1)

status_sub = rospy.Subscriber('/'+drone_id+'/status', DroneStatus, update_status)
package_sub = rospy.Subscriber('/'+drone_id+'/assign', DronePackage, assign_package)

rate = rospy.Rate(30) # 10hz

height = 0.6
takeoff_velocity = 0.3
land_velocity = 0.7
move_velocity = 0.35

cmd_msg = DroneCommand()
cmd_msg.drone_id = drone_id
cmd_msg.z = height
cmd_msg.yaw = 0
cmd_msg.velocity = move_velocity

land_msg = DroneLandTakeoff()
land_msg.drone_id = drone_id
land_msg.height = 0
land_msg.velocity = land_velocity

takeoff_msg = DroneLandTakeoff()
takeoff_msg.drone_id = drone_id
takeoff_msg.height = height/2
takeoff_msg.velocity = takeoff_velocity

print "Ready"
while not rospy.is_shutdown():
    for i in range(len(package_list)):
        if package_status[i] == True:
            continue

        ### Assume you are at home

        ### Plan and Take off
        print "Plan path to package location"
        print curr,(package_list[i].x,package_list[i].y)

        print "Sending takeoff command"
        path_1 = plan(curr,(package_list[i].x,package_list[i].y))
        takeoff_pub.publish(takeoff_msg)

        rospy.sleep(LAND_TIME)

        ### Do magic
        goToPath(path_1)
        rospy.sleep(1)

        ### land
        print "Sending land command"
        land_pub.publish(land_msg)

        rospy.sleep(LAND_TIME)

        ### Plan and Take off
        print "Plan path to home location"
        print curr,home

        print "Sending takeoff command"
        path_2 = plan(curr,home)
        takeoff_pub.publish(takeoff_msg)

        rospy.sleep(LAND_TIME)

        ### plan

        ### do magic
        goToPath(path_2)
        rospy.sleep(1)

        ### land
        print "Sending land command"
        land_pub.publish(land_msg)

        rospy.sleep(LAND_TIME)

        # Update that you dropped
        # Tell server you dropped
        package_status[i] = True
        delivered_pub.publish(package_list[i])
