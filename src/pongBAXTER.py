#!/usr/bin/env python
import rospy
'''
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
'''
import baxter_interface
from geometry_msgs.msg import Twist
import modern_robotics as mr
import numpy as np
import random

# test function publish velocity commands in yz plane
def main():
    try:
        rospy.init_node("pong")
        velpub = rospy.Publisher("pongvelocity", Twist, queue_size=1)
        # specify rate
        rate = rospy.Rate(0.2)

        while not rospy.is_shutdown():
            #create the Twist message
            vel = Twist()
            # set linear velocities
            vel.linear.x = random.uniform(0,0.3)
            vel.linear.y = random.uniform(0,0.3)
            vel.linear.z = random.uniform(0,0.3)
            # set the angular speeds
            vel.angular.x = random.uniform(0,0.15)
            vel.angular.y = random.uniform(0,0.15)
            vel.angular.z = random.uniform(0,0.15)
            # publish the twist
            velpub.publish(vel)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr('Failed to create velocities.')


if __name__=="__main__":
    main()
