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

def publishTwist(publisher, veltwist, boundxhit=0, boundzhit=0):

    if boundxhit == 1:
        # set linear velocities
        veltwist.linear.x = -1*abs(veltwist.linear.x)
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

    if abs(posemsg.position.x) >= abs(0.3):
        oob_x = 1
        #print("Reflection")
    # if abs(posemsg.position.z) >= abs(0.15):
    #     oob_z = 1
    #     print("Reflection")

# test function publish velocity commands in yz plane
def main():
    global oob_x
    global oob_z
    #global run1

    vel = Twist()
    vel.linear.y = 0.0
    vel.linear.x = 0.1
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0

    # can_update = 1

    try:
        rospy.init_node("pong")
        velpub = rospy.Publisher("pongvelocity", Twist, queue_size=1)
        # specify rate
        rate = rospy.Rate(1)
        # publishTwist(velpub,vel,oob_x,oob_z)
        # print("Vel.linear.x:")
        # print(vel.linear.x)
        # rospy.sleep(5)

        while not rospy.is_shutdown():


            publishTwist(velpub,vel,oob_x,oob_z)
            # if oob_x == 0:
            #     can_update == 1


            poscheck = rospy.Subscriber("endpoint_Pose", Pose, checkPosition)
            if (oob_x == 1):
                print("Vel.linear.x:")
                print(vel.linear.x)
                publishTwist(velpub,vel,1,0)
                oob_x = 0
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
