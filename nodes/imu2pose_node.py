#!/usr/bin/env python

import rospy
import math
import sys
import tf

#from time import time
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

degrees2rad = math.pi/180.0
rad2deg = 180.0/math.pi

rospy.init_node("imu_2_pose_node")
imuTopic = rospy.get_param('~imu_topic', 'imu/data/filtered')
poseTopic = rospy.get_param('~pose_topic', 'imu/pose/filtered')
verbose = rospy.get_param('~verbose', False)

posepub = rospy.Publisher(poseTopic, PoseStamped, queue_size=1)

def imuCallback(imuMsg):
    global verbose
    poseMsg = PoseStamped()
    poseMsg.header = imuMsg.header
    poseMsg.pose.position.x = poseMsg.pose.position.y = poseMsg.pose.position.z = 0.0
    poseMsg.pose.orientation = imuMsg.orientation
    posepub.publish(poseMsg)
    if(verbose):
        quats = (poseMsg.pose.orientation.x, poseMsg.pose.orientation.y,poseMsg.pose.orientation.z,poseMsg.pose.orientation.w)
        euler = euler_from_quaternion(quats)
        r = rad2deg*euler[0]
        p = rad2deg*euler[1]
        y = rad2deg*euler[2]
        print("[INFO] imu2pose_node.py ---- filtered euler angles (R,P,Y) [deg]: %.2f, %.2f, %.2f" % (r,p,y))

sub = rospy.Subscriber(imuTopic, Imu, imuCallback)
rospy.spin()
