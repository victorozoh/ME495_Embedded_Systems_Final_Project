#!/usr/bin/env python
import rospy
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
from intera_interface import Limb
from geometry_msgs import Twist
import modern_robotics as mr

# test function publish velocity commands in yz plane
def main():
    try:
        rospy.init_node("pong")
        velpub = rospy.Publisher("pongvelocity", Twist, queue_size=1)
        # specify rate
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            #create the Twist message
            vel = Twist()
            # set linear velocities
            vel.linear.x = 0
            vel.linear.y = 0.10
            vel.linear.z = 0.10
            # set the angular speeds
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            # publish the twist
            velpub.Publish(vel)
            rate.sleep()
    except ROSInterruptException:
        rospy.logerr('Failed to create velocities.')


if __name__=="__main__":
    main()
