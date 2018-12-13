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
# get end effector velocity from pong logi

def main():
    try:
        rospy.init_node("getconfig")
        limb = Limb()
        joint_vels_dict = {}
        for i in range(7):
            joint_vels_dict['right_j'+ str(i)] = 0.01
        print joint_vels_dict


        for j in range(1000):



            limb.set_joint_velocities(joint_vels_dict)
            endpoint_pose = limb.endpoint_pose()
            # get current pose
            #print("The current end effector pose is {}".format(endpoint_pose))
            # get all current joint angles
            # print(limb.joint_angles())
            # print(type(limb.joint_angles()))
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        rospy.logerr('Could not get the requested info.')

if __name__=="__main__":
    main()
