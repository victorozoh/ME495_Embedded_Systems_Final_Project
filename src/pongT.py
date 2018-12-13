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
oob_x = 0
oob_z = 0
# get Slist from Jarvis description file
import sawyer_MR_description as sw


def move(velocity):
    #global endpoint_Pose
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
    print("made it into move")
    rospy.loginfo(thetadict)
    thetalist = []

    Blist = sw.Blist
    M = sw.M
    Slist = mr.Adjoint(M).dot(Blist)
    thetalist.append(thetadict['right_j0'])
    thetalist.append(thetadict['right_j1'])
    thetalist.append(thetadict['right_j2'])
    thetalist.append(thetadict['right_j3'])
    thetalist.append(thetadict['right_j4'])
    thetalist.append(thetadict['right_j5'])
    thetalist.append(thetadict['right_j6'])

    '''for i in range(len(thetadict)):
        thetalist.append(thetadict['right_j'+ str(i)])'''
    thetalist = np.array(thetalist)
    # J = mr.JacobianBody(Blist, thetalist)
    J = mr.JacobianSpace(Slist, thetalist)
    pinv_J = np.linalg.pinv(J)
    print("The shape of the end effector velocity vector is {}".format(end_effector_vel.shape))
    print("The shape of the Jacobian Pseudo Inverse matrix is {}".format(pinv_J.shape))
    joint_vels = np.dot(pinv_J,end_effector_vel)
    # velocities need to be passed in as a dictionary
    joint_vels_dict = {}


    for i in range(len(joint_vels)):
        joint_vels_dict['right_j'+ str(i)] = joint_vels[i]

    limb.set_joint_velocities(joint_vels_dict)
    endpoint_Pose =limb.endpoint_pose()
    #print(endpoint_Pose)

    currPose = Pose()
    currPose.position = endpoint_Pose['position']
    currPose.orientation = endpoint_Pose['orientation']

    callback_pub = rospy.Publisher("endpoint_Pose", Pose, queue_size=1)
    callback_pub.publish(currPose)

def move_to_home(thetalist):
    rightlimb = Limb()
    waypoints = {}
    waypoints['right_j0'] = thetalist[0]
    waypoints['right_j1'] = thetalist[1]
    waypoints['right_j2'] = thetalist[2]
    waypoints['right_j3'] = thetalist[3]
    waypoints['right_j4'] = thetalist[4]
    waypoints['right_j5'] = thetalist[5]
    waypoints['right_j6'] = thetalist[6]
    waypoints['torso_t0'] = thetalist[7]
    rightlimb.move_to_joint_positions(waypoints, timeout = 20.0, threshold = 0.05)

def publishTwist(publisher, veltwist, boundxhit=0, boundzhit=0):

    if boundxhit == 1:
        # set linear velocities
        veltwist.linear.x = -1*abs(veltwist.linear.x)

    if boundxhit == 2:
        # set linear velocities
        veltwist.linear.x = abs(veltwist.linear.x)

    if boundzhit == 1:
        # set linear velocities
        veltwist.linear.z = -1*abs(veltwist.linear.z)

    if boundzhit == 2:
        # set linear velocities
        veltwist.linear.z = abs(veltwist.linear.z)
    # publish the twist
    move(veltwist)



'''
def publishTwist(publisher, boundyhit=0, boundzhit=0):
    #create the Twist message
    vel = Twist()
    # set linear velocities
    vel.linear.x = 0#np.random.uniform(0,0.005)
    vel.linear.z = 0
    vel.linear.y = 0.05#np.random.uniform(0,0.005)
    if boundyhit == 1:
        vel.linear.y = -0.05#np.random.uniform(0,0.005)
    #if boundzhit == 0:
        #vel.linear.z = 0#np.random.uniform(0,0.005)
    #if boundzhit == 1:
        #vel.linear.z = 0#-np.random.uniform(0,0.005)
    # set the angular speeds
    vel.angular.x = 0#np.random.uniform(0,0.005)
    vel.angular.y = 0#np.random.uniform(0,0.005)
    vel.angular.z = 0#np.random.uniform(0,0.005)
    # publish the twist
    publisher.publish(vel)
'''

def checkPosition(posemsg):
    global oob_x
    global oob_z

    #print('this is x pose:' + str(posemsg.position.x))
    #print('this is z pose:' + str(posemsg.position.z))

    if posemsg.position.x >= 0.425:
        oob_x = 1
    if posemsg.position.x <= -0.275: #-0.335:
        oob_x = 2
        #print("Reflection")
    if posemsg.position.z >= 0.44:
        oob_z = 1
    if posemsg.position.z <= 0.24:
        oob_z = 2
    #     print("Reflection")

# test function publish velocity commands in yz plane
def main():
    global oob_x
    global oob_z
    #global run1


    vel = Twist()
    vel.linear.y = 0.0
    vel.linear.x = 0.075#0.0
    vel.linear.z = 0.030#0.0
    vel.angular.x = 0.0
    vel.angular.y = 0.0
    vel.angular.z = 0.0

    # can_update = 1

    try:
        rospy.init_node("pong")
        thetalistHOME = [1.3395771484375, -3.5355, 2.0365224609375, -1.489580078125, -0.4218515625, 1.1975029296875, -3.419748046875, 0.0]
        move_to_home(thetalistHOME)
        velpub = rospy.Publisher("pongvelocity", Twist, queue_size=1)
        # specify rate
        rate = rospy.Rate(100)


        # publishTwist(velpub,vel,oob_x,oob_z)
        # print("Vel.linear.x:")
        # print(vel.linear.x)
        # rospy.sleep(5)

        while not rospy.is_shutdown():


            publishTwist(velpub,vel,oob_x,oob_z)
            # if oob_x == 0:
            #     can_update == 1


            poscheck = rospy.Subscriber("endpoint_Pose", Pose, checkPosition, queue_size=1)
            if (oob_x == 1):
                print("Vel.linear.x:")
                print(vel.linear.x)
                publishTwist(velpub,vel,1,0)
                oob_x = 0
            if (oob_x == 2):
                print("Vel.linear.x LEFT:")
                print(vel.linear.x)
                publishTwist(velpub,vel,2,0)
                oob_x = 0
            if (oob_z == 1):
                print("Vel.linear.z:")
                print(vel.linear.z)
                publishTwist(velpub,vel,0,1)
                oob_z = 0
            if (oob_z == 2):
                print("Vel.linear.z:")
                print(vel.linear.z)
                publishTwist(velpub,vel,0,2)
                oob_z = 0
                # can_update = 0
            #else:
                #publishTwist(velpub,vel,0,0)
            #print(oob_z)
            # if oob_z == 1:
            #     publishTwist(velpub,0,1)
            #     oob_z = 0

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr('Failed to create velocities.')


if __name__=="__main__":
    main()
