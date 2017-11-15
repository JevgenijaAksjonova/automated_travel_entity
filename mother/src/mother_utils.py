#! /usr/bin/env python
from __future__ import print_function
import sys
import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0,rospack.get_path("mother"))
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, PointStamped, Quaternion, Point, Pose, Twist, TransformStamped
from tf.transformations import quaternion_from_euler, vector_norm
from mother_settings import *

def pose_to_msg(x,y,theta,z=0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quat = quaternion_from_euler(0,0,theta)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def point_to_msg(x,y):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0
    return point

def pose_to_msg_stamped(x,y,theta,z=0):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose_to_msg(x,y,theta,z)
    pose_stamped.header.frame_id = MOTHER_WORKING_FRAME
    pose_stamped.header.stamp = rospy.Time.now()
    return pose_stamped
