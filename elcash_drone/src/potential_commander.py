#!/usr/bin/env python
import rospy
from elcash_drone.msg import DroneStatus, DroneLandTakeoff, DroneCommand
from elcash_drone.msg import DronePackage
import networkx as nx
import potentialFieldPlanning

LAND_TIME = 3


def plan(origin_pos, goal_pos):
    origin = best_match(origin_pos, NODES)
    target = best_match(goal_pos, NODES)
    return path(origin, target)

package_list = []
package_status = []

THRESH = 0.1

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
        target = (package_list[i].x,package_list[i].y)

        print "Sending takeoff command"
        takeoff_pub.publish(takeoff_msg)
        rospy.sleep(LAND_TIME)

        ### do magic
        while distance(curr, target) > THRESH:
            obstacles = [(2, 2.8), (1.5, 1), (3.15, 0.7), (3, 2)]
            (x, y) = potentialFieldPlanning.next_pos(curr, target, obstacles)
            cmd_msg.x = x
            cmd_msg.y = y
            cmd_pub.publish(cmd_msg)
            rospy.sleep(1)

        ### land
        print "Sending land command"
        land_pub.publish(land_msg)

        rospy.sleep(LAND_TIME)

        ### Plan and Take off
        print "Plan path to home location"
        print curr,home
        path_2 = plan(curr,home)

        print "Sending takeoff command"

        takeoff_pub.publish(takeoff_msg)
        rospy.sleep(LAND_TIME/2)

        while dist(curr, target) > THRESH:
            obstacles = [(2, 2.8), (1.5, 1), (3.15, 0.7), (3, 2)]
            (x, y) = potentialFieldPlanning.next_pos(curr, target, obstacles)
            cmd_msg.x = x
            cmd_msg.y = y
            cmd_pub.publish(cmd_msg)
            rospy.sleep(1)

        ### land
        print "Sending land command"
        land_pub.publish(land_msg)

        rospy.sleep(LAND_TIME)

        # Update that you dropped
        # Tell server you dropped
        package_status[i] = True
        delivered_pub.publish(package_list[i])
