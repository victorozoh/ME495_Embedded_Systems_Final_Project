#! /usr/bin/env python
import sys
import rospy
import numpy as np
from math import ceil
import argparse
import actionlib
import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import (PoseStamped, Point)
from tf_conversions import posemath
from intera_interface import Limb


def movetopoint(x, y, z):
    print("goto_cartesian called with x={}, y={}, z={}".format(x,y,z))
    limb = Limb()
    traj_options = TrajectoryOptions()
    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
    traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

    wpt_opts = MotionWaypointOptions()
    waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

    joint_names = limb.joint_names()
    endpoint_state = limb.tip_state('right_hand')
    if endpoint_state is None:
        print('Endpoint state not found')
        return self.MOVE_ERROR
        pose = endpoint_state.pose

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        # pose.orientation.x = None
        # pose.orientation.y = self.orientation_y
        # pose.orientation.z = self.orientation_z
        # pose.orientation.w = self.orientation_w
        poseStamped = PoseStamped()
        poseStamped.pose = pose
        waypoint.set_cartesian_pose(poseStamped, 'right_hand', [])

        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory()
        if result is None:
            print("Trajectory FAILED to send")
            return self.MOVE_ERROR

        if result.result:
            print('Motion controller successfully finished the trajectory!')
            # self.cur_x = x
            # self.cur_y = y
            # self.cur_z = z
            return self.MOVE_SUCCESS
        else:
            print('Motion controller failed to complete the trajectory %s',
                  result.errorId)
            return self.MOVE_ERROR


def main():
    try:
        rospy.init_node("movetopoint")
        q = [np.random.uniform(0,1.0) for i in range(3)]
        q = tuple(np.round(q,3))
        movetopoint(*q)
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


if __name__ == "__main__":
    main()
