#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
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
    # have to switch the order of linear and angular velocities in twist
    # message so that it comes in the form needed by the modern_robotics library
    end_effector_vel = np.zeros(6)
    end_effector_vel[0] = velocity.angular.x
    end_effector_vel[1] = velocity.angular.y
    end_effector_vel[2] = velocity.angular.z
    end_effector_vel[3] = velocity.linear.x
    end_effector_vel[4] = velocity.linear.y
    end_effector_vel[5] = velocity.linear.z
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
    joint_vels = pinv_J.dot(end_effector_vel)
    # velocities need to be passed in as a dictionary
    joint_vels_dict = {}
    for i in range(len(joint_vels)):
        joint_vels_dict['right_j'+ str(i)] = joint_vels[i]
    limb.set_joint_velocities(joint_vels_dict)


def main():
    try:
        rospy.init_node("armcontrol")
        # subscribe to pong velocities published by pong node
        sub = rospy.Subscriber('pongvelocity', Twist, move)
        rospy.spin()
    except ROSInterruptException:
        rospy.logerr('Could not perform the requested motion.')

if __name__=="__main__":
    main()
