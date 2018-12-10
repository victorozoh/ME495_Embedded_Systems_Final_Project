#!/usr/bin/env python
import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_interface import Limb

import modern_robotics as mr
# get end effector velocity from pong logic
def move(velocity):
    # we'll treat the velocity vector as a twist 6x1
    # 1. Compute the Jacobian. Get the Blist and thetalist
    limb = Limb()
    # limb.joint_angles() returns a dictionary
    thetadict = limb.joint_angles()
    thetalist = []
    M = np.array([[0, 0, 1, 1.0039],
                  [1, 0, 0, 0.1603],
                  [0, 1, 0, 0.317],
                  [0, 0, 0, 1]])
    Blist = np.array([[0, 1, 0, -1, 0, 1, 0],
                      [1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 1, 0, 1],
                      [1.0039, 0, -0.0322, 0, 0, 0, 0],
                      [0, -0.920, 0, 0.520, 0.1363, -0.120, 0],
                      [-0.1603, 0, 0, 0, 0, 0, 0]).T
    for i in range(len(thetadict)):
        thetalist.append(thetadict['right_j'+ str(i)])
    thetalist = np.array(thetalist)
    J = mr.JacobianBody(Blist, thetalist)
    pinv_J = np.linalg.pinv(J)
    joint_vels = pinv_J.dot(velocity)
    limb.set_joint_velocities(joint_vels)



def main():
    try:
        rospy.init_node("getconfig")
        limb = Limb()
        # get current pose
        print(limb.endpoint_pose())
        # get all current joint angles
        print(limb.joint_angles())
        print(type(limb.joint_angles()))
    except ROSInterruptException:
        rospy.logerr('Could not get the requested info.')

if __name__=="__main__":
    main()
