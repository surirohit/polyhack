#!/usr/bin/env python
import rospy
from elcash_drone.msg import DroneStatus, DroneLandTakeoff, DroneCommand

if __name__ == '__main__':
    drone_id = rospy.get_param('~drone_id', 'drone_11')
    rospy.init_node(drone_id+"_commander", anonymous=False)

    cmd_pub = rospy.Publisher(drone_id+'/goto', DroneCommand, queue_size=1)
    land_pub = rospy.Publisher(drone_id+'/land', DroneLandTakeoff, queue_size=1)
    takeoff_pub = rospy.Publisher(drone_id+'/takeoff', DroneLandTakeoff, queue_size=1)

    rate = rospy.Rate(30) # 10hz

    height = 0.5
    takeoff_velocity = 0.7
    land_velocity = 0.7
    move_velocity = 0.7

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

    x = [2,3,2,1,1]
    y = [2,3,4,3,2]
    for x_i,y_i in zip(x,y):
        print "Sending", x_i,y_i,"command"
        cmd_msg.x = x_i
        cmd_msg.y = y_i
        cmd_msg.z = height
        cmd_pub.publish(cmd_msg)
        rospy.sleep(5)

    ## Tell it to land and exit
    print "Sending land command"
    land_pub.publish(land_msg)

    rospy.spin()
