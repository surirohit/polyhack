#!/usr/bin/env python
import rospy
from elcash_drone.msg import DroneStatus, DroneLandTakeoff, DroneCommand
from elcash_drone.msg import DronePackage
import networkx as nx
import potentialFieldPlanning

LAND_TIME = 3

OBSTACLES = [(2, 2.8), (1.5, 1), (3.1, 0.7), (3, 2)]
drone = ['drone_11', 'drone_12','drone_13']

package_list = []
package_status = []

THRESH = 0.1

curr = (0,0)
home = (2.2,1.6) #TODO

other_1 = (0,0)
other_2 = (0,0)

def update_status_1(msg):
    global other_1
    other_1 = (msg.x,msg.y)

def update_status_2(msg):
    global other_2
    other_2 = (msg.x,msg.y)

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

rospy.init_node("commander", anonymous=True)
drone_id = rospy.get_param('~drone_id', 'drone_11')

cmd_pub = rospy.Publisher(drone_id+'/goto', DroneCommand, queue_size=1)
land_pub = rospy.Publisher(drone_id+'/land', DroneLandTakeoff, queue_size=1)
takeoff_pub = rospy.Publisher(drone_id+'/takeoff', DroneLandTakeoff, queue_size=1)
delivered_pub = rospy.Publisher('/delivered', DronePackage, queue_size=1)

not_me = [d for d in drone if d!=drone_id]

status_sub_1 = rospy.Subscriber('/'+not_me[0]+'/status', DroneStatus, update_status_1)
status_sub_2 = rospy.Subscriber('/'+not_me[1]+'/status', DroneStatus, update_status_2)

status_sub = rospy.Subscriber('/'+drone_id+'/status', DroneStatus, update_status)
package_sub = rospy.Subscriber('/'+drone_id+'/assign', DronePackage, assign_package)

rate = rospy.Rate(30) # 10hz

height = 0.4
takeoff_velocity = 3
land_velocity = 0.7
move_velocity = 1

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
            obs = list(OBSTACLES)
            if other_1 != (0,0):
                obs = OBSTACLES + [other_1]
            if other_2 != (0,0):
                obs = OBSTACLES + [other_2]

            (x, y) = potentialFieldPlanning.next_pos(curr, target, obs)
            cmd_msg.x = x
            cmd_msg.y = y
            cmd_pub.publish(cmd_msg)
            rospy.sleep(1)

        rospy.sleep(2)

        ### land
        print "Sending land command"
        land_pub.publish(land_msg)

        rospy.sleep(LAND_TIME*0.75)

        ### Plan and Take off
        print "Plan path to home location"
        print curr,home

        print "Sending takeoff command"

        takeoff_pub.publish(takeoff_msg)
        rospy.sleep(LAND_TIME/2)

        while distance(curr, home) > THRESH:
            obs = list(OBSTACLES)
            if other_1 != (0,0):
                obs = OBSTACLES + [other_1]
            if other_2 != (0,0):
                obs = OBSTACLES + [other_2]

            (x, y) = potentialFieldPlanning.next_pos(curr, home, obs)
            cmd_msg.x = x
            cmd_msg.y = y
            cmd_pub.publish(cmd_msg)
            rospy.sleep(1)

        rospy.sleep(2)

        ### land
        print "Sending land command"
        land_pub.publish(land_msg)

        rospy.sleep(LAND_TIME*0.75)

        # Update that you dropped
        # Tell server you dropped
        package_status[i] = True
        delivered_pub.publish(package_list[i])
