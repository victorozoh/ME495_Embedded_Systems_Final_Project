#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
'''
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
'''
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
import baxter_interface #import Limb

import numpy as np
import modern_robotics as mr
# get end effector velocity from pong logic

def move(velocity):
    print("made it to MOVE()")
    '''
    armpub = rospy.Publisher('pongarmcontrol',Twist,queue_size=10)
    rate = rospy.Rate(0.2)
    '''
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", velocity.velocity)
    # have to switch the order of linear and angular velocities in twist
    # message so that it comes in the form needed by the modern_robotics library
    '''
    end_effector_vel = np.zeros(6)
    end_effector_vel[0] = Twist.angular.x
    end_effector_vel[1] = Twist.angular.y
    end_effector_vel[2] = Twist.angular.z
    end_effector_vel[3] = Twist.linear.x
    end_effector_vel[4] = Twist.linear.y
    end_effector_vel[5] = Twist.linear.z
    '''
    end_effector_vel = np.zeros(6)
    end_effector_vel[0] = velocity.angular.x
    end_effector_vel[1] = velocity.angular.y
    end_effector_vel[2] = velocity.angular.z
    end_effector_vel[3] = velocity.linear.x
    end_effector_vel[4] = velocity.linear.y
    end_effector_vel[5] = velocity.linear.z

    # we'll treat the velocity vector as a twist 6x1
    # 1. Compute the Jacobian. Get the Blist and thetalist
    leftlimb = baxter_interface.limb.Limb("left")
    # limb.joint_angles() returns a dictionary
    thetadict = leftlimb.joint_angles()
    print(thetadict)
    thetalist = []

        ######################################
    # LEFT ARM SPATIAL SCREWS WRT /base #
    ######################################
    # Joint 1
    w1 = np.array([0,0,1])
    q1 = np.array([0.06402724, 0.25902738, 0])
    v1 = -np.cross(w1,q1)
    S1 = np.append(w1,v1)
    # Joint 2
    w2 = np.array([-sq2, sq2, 0])
    q2 = np.array([0.11281752, 0.30781784, 0.399976])
    v2 = -np.cross(w2,q2)
    S2 = np.append(w2,v2)
    # Joint 3
    w3 = np.array([sq2, sq2, 0])
    q3 = np.array([0.18494228, 0.37994287, 0.399976])
    v3 = -np.cross(w3,q3)
    S3 = np.append(w3,v3)
    # Joint 4
    w4 = np.array([-sq2, sq2, 0])
    q4 = np.array([0.3705009, 0.56550217, 0.330976])
    v4 = -np.cross(w4,q4)
    S4 = np.append(w4,v4)
    # Joint 5
    w5 = np.array([sq2, sq2, 0])
    q5 = np.array([0.44374996, 0.63875149, 0.330976])
    v5 = -np.cross(w5,q5)
    S5 = np.append(w5,v5)
    # Joint 6
    w6 = np.array([-sq2, sq2, 0])
    q6 = np.array([0.63516341, 0.83016565, 0.320976])
    v6 = -np.cross(w6,q6)
    S6 = np.append(w6,v6)
    # Joint 7
    w7 = np.array([sq2, sq2, 0])
    q7 = np.array([0.71716997, 0.91217251, 0.320976])
    v7 = -np.cross(w7,q7)
    S7 = np.append(w7,v7)

    # assemble screws:
    joints = [S1, S2, S3, S4, S5, S6, S7]
    Slist_left = np.array(joints).T
    M0_blh = np.zeros((4,4))
    M0_blh[0:3,0:3] = np.array([[ 0, -sq2, sq2],
                                [ 0,  sq2, sq2],
                            [-1,  0,   0]])
M0_blh[0:3,-1] = np.array([0.7974618, 0.99246463, 0.320976])
M0_blh[3,3] = 1


###############################
# BODY SCREWS WRT /left_hand #
###############################
M0_lhb = mr.TransInv(M0_blh)
Blist_left = np.zeros(Slist_left.shape)
for i,s in enumerate(Slist_left.T):
Blist_left[:,i] = np.dot(mr.Adjoint(M0_lhb), s)

    '''
    M = np.array([[0, 0, 1, 1.0039],
                  [1, 0, 0, 0.1603],
                  [0, 1, 0, 0.317],
                  [0, 0, 0, 1]])
    Blist = np.array([[0, 1, 0, -1, 0, 1, 0],
                      [1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 1, 0, 1],
                      [1.0039, 0, -0.0322, 0, 0, 0, 0],
                      [0, -0.920, 0, 0.520, 0.1363, -0.120, 0],
                      [-0.1603, 0, 0, 0, 0, 0, 0]]).T
    '''
    thetalist.append(thetadict['left_w0'])
    thetalist.append(thetadict['left_w1'])
    thetalist.append(thetadict['left_w2'])
    thetalist.append(thetadict['left_e0'])
    thetalist.append(thetadict['left_e1'])
    thetalist.append(thetadict['left_s0'])
    thetalist.append(thetadict['left_s1'])

    '''
    for i in range(len(thetadict)):
        thetalist.append(thetadict['left_j'+ str(i)])
    '''
    thetalist = np.array(thetalist)
    J = mr.JacobianBody(Blist, thetalist)
    pinv_J = np.linalg.pinv(J)
    joint_vels = np.dot(pinv_J,end_effector_vel)
    # velocities need to be passed in as a dictionary
    joint_vels_dict = {}

    joint_vels_dict['left_w0'] = joint_vels[0]
    joint_vels_dict['left_w1'] = joint_vels[1]
    joint_vels_dict['left_w2'] = joint_vels[2]
    joint_vels_dict['left_e0'] = joint_vels[3]
    joint_vels_dict['left_e1'] = joint_vels[4]
    joint_vels_dict['left_s0'] = joint_vels[5]
    joint_vels_dict['left_s1'] = joint_vels[6]

    '''
    for i in range(len(joint_vels)):
        joint_vels_dict['left_j'+ str(i)] = joint_vels[i]
    '''

    leftlimb.set_joint_velocities(joint_vels_dict)
    #armpub.publish(leftlimb)
    print(leftlimb)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.linear.x)


def main():
#try:
    rospy.init_node("armcontrol")
    # subscribe to pong velocities published by pong node
    rospy.Subscriber("pongvelocity", Twist, move)
    #rate = rospy.Rate(1);
    #armpub = rospy.Publisher('pongarmcontrol',Twist,queue_size=10)
    #rate = rospy.Rate(0.2)
    #armpub.publish(rate)
    print("MADE IT TO SPIN")
    rospy.spin()
#    except rospy.ROSInterruptException:
    #rospy.logerr('Could not perform the requested motion.')

if __name__=="__main__":
    main()
