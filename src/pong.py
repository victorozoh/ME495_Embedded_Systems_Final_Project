#!/usr/bin/env python
import rospy
import numpy as np
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
from intera_interface import Limb
from geometry_msgs.msg import (
    Twist,
    Pose
    )
import modern_robotics as mr
oob_y = 0
oob_z = 0

def publishTwist(publisher):
    #create the Twist message
    vel = Twist()
    # set linear velocities
    vel.linear.x = np.random.uniform(0,0.005)
    vel.linear.y = np.random.uniform(0,0.005)
    vel.linear.z = np.random.uniform(0,0.005)
    # set the angular speeds
    vel.angular.x = np.random.uniform(0,0.005)
    vel.angular.y = np.random.uniform(0,0.005)
    vel.angular.z = np.random.uniform(0,0.005)
    # publish the twist
    publisher.publish(vel)

def checkPosition(posemsg):
    global oob_y
    global oob_z

    print(posemsg.position.y)
    print(posemsg.position.z)

    if abs(posemsg.position.y) >= abs(0.17):
        oob_y = 1
    if abs(posemsg.position.z) >= abs(0.25):
        oob_z = 1

# test function publish velocity commands in yz plane
def main():
    global oob_y
    global oob_z
    try:
        rospy.init_node("pong")
        velpub = rospy.Publisher("pongvelocity", Twist, queue_size=1)
        # specify rate
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            publishTwist(velpub)
            poscheck = rospy.Subscriber("endpoint_Pose", Pose, checkPosition)
            print(oob_y)
            print(oob_z)
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr('Failed to create velocities.')


if __name__=="__main__":
    main()
