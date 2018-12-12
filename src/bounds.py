#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
# from intera_motion_interface import (
#     MotionTrajectory,
#     MotionWaypoint,
#     MotionWaypointOptions
# )
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
from tf_conversions import posemath
from intera_interface import Limb
import modern_robotics as mr
from tf.transformations import quaternion_matrix
import math



def main():
    try:
        rospy.init_node("bounds")
        velpub = rospy.Publisher("velocities", Twist, queue_size=1)
        # specify rate
        rate = rospy.Rate(20)
        # specify the boundaries. But first get the end effector pose and preserve orientation
        limb = Limb()
        endpoint_pose =limb.endpoint_pose()
        position = list(endpoint_pose['position'])
        # convert position to numpy array for easy arithmetic
        position = np.array(position)
        orientation = list(endpoint_pose['orientation'])
        rot_matrix = quaternion_matrix(orientation)
        # the above returns a 4x4 matrix. perform slicing to get the rotational component
        rot_matrix = rot_matrix[:-1,:-1]
        # point 1 . -0.17 offset in y and 0.25 offset in z. top left corner
        p1 = position + np.array([0, -0.17, 0.25])
        # point 1 . 0.17 offset in y and 0.25 offset in z,top left corner
        p2 = position + np.array([0, 0.17, 0.25])
        # point 1 . -0.17 offset in y and -0.25 offset in z, bottom left corner
        p3 = position + np.array([0, -0.17, -0.25])
        # point 1 . 0.17 offset in y and -0.25 offset in z, bottom right corner
        p4 = position + np.array([0, 0.17, -0.25])
        # convert positions into homogenous matrices for for input into cartesian trajectory
        X1 = mr.RpToTrans(rot_matrix, p1)
        X2 = mr.RpToTrans(rot_matrix, p2)
        X3 = mr.RpToTrans(rot_matrix, p3)
        X4 = mr.RpToTrans(rot_matrix, p4)
        # perform interpolation between points
        Tf = 5
        p1_p2 = mr.CartesianTrajectory(X1, X2,Tf,100,3)
        p2_p3 = mr.CartesianTrajectory(X2, X3,Tf,100,3)
        p3_p4 = mr.CartesianTrajectory(X3, X4,Tf,100,3)
        p4_p1 = mr.CartesianTrajectory(X4, X1,Tf,100,3)
        # may be simpler to use the intersection of two lines
        # to determine where end effector will meet the boundary


        while not rospy.is_shutdown():
            #create the Twist message
            vel = Twist()
            # set linear velocities
            vel.linear.x = 0
            vel.linear.y = -0.06
            vel.linear.z = -0.06
            # set the angular speeds
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            # publish the twist
            velpub.publish(vel)
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("failed to define bounds")

if __name__=="__main__":
    main()
