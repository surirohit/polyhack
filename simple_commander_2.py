#!/usr/bin/env python
import rospy
from elcash_drone.msg import DroneStatus, DroneLandTakeoff, DroneCommand
import SimplePlanPlanning.py

if __name__ == '__main__':
    drone_id = rospy.get_param('~drone_id', 'drone_11')
    rospy.init_node(drone_id+"_commander", anonymous=False)

    cmd_pub = rospy.Publisher(drone_id+'/goto', DroneCommand, queue_size=1)
    land_pub = rospy.Publisher(drone_id+'/land', DroneLandTakeoff, queue_size=1)
    takeoff_pub = rospy.Publisher(drone_id+'/takeoff', DroneLandTakeoff, queue_size=1)

    rate = rospy.Rate(10) # 10hz

    height = 1
    takeoff_velocity = 0.2
    land_velocity = 0.2
    move_velocity = 0.2

    cmd_msg = DroneCommand()
    cmd_msg.drone_id = drone_id
    cmd_msg.yaw = 0
    cmd_msg.velocity = move_velocity

    land_msg = DroneLandTakeoff()
    land_msg.drone_id = drone_id
    land_msg.height = 0
    land_msg.velocity = land_velocity

    takeoff_msg = DroneLandTakeoff()
    takeoff_msg.drone_id = drone_id
    takeoff_msg.height = height
    takeoff_msg.velocity = takeoff_velocity

    ## Wait for stuff to come online
    rospy.sleep(5)

    ## Tell it to take off and wait
    print "Sending takeoff command"
    takeoff_pub.publish(takeoff_msg)
    rospy.sleep(5)

    ## Tell it to go to (0,0) and wait till it reaches
    print "Sending 0,0 command"
    path = path_setting(cx,cy,dx,dy)
    path_array = path_planning_simple(path)
    i=0
    if len(path_array)>1:
        for i in len(path_array):
            while (sqrt((cx-path_array[i][0])**2+(cy-path_array[i][1])**2))<0.2:
                cmd_msg.x = path_array[i][0]
                cmd_msg.y = path_array[i][1]
                cmd_msg.z = height
                cmd_pub.publish(cmd_msg)
    else:
        cmd_msg.x = path_array[0][0]
        cmd_msg.y = path_array[0][1]
        cmd_msg.z = height
        cmd_pub.publish(cmd_msg)
    
    

    ## Tell it to go to (0,0) and wait till it reaches
    print "Sending 1,1 command"
    cmd_msg.x = 0
    cmd_msg.y = 0
    cmd_msg.z = height
    cmd_pub.publish(cmd_msg)
    rospy.sleep(5)

    ## Tell it to land and exit
    print "Sending land command"
    land_pub.publish(land_msg)

    rospy.spin()
