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
    publisher.publish(veltwist)



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
