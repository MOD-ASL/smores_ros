#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
import time
from math import pi
import tf


rospy.init_node('pose_pub', anonymous = True)
pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)

def viewPoint():

    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.8
    pose.pose.position.z = 0.0
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, -45.0/180*pi)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    pose.header.frame_id = "/map"
    pose.header.stamp = rospy.Time.now()
    pub.publish(pose)

def rampPoint():

    pose = PoseStamped()
    pose.pose.position.x = 0.65
    pose.pose.position.y = 0.4
    pose.pose.position.z = 0.0
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0/180*pi)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    pose.header.frame_id = "/map"
    pose.header.stamp = rospy.Time.now()
    pub.publish(pose)

def placePoint():

    pose = PoseStamped()
    pose.pose.position.x = 0.146
    pose.pose.position.y = -0.20
    pose.pose.position.z = 0.0
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, -45.0/180*pi)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    pose.header.frame_id = "/map"
    pose.header.stamp = rospy.Time.now()
    pub.publish(pose)
