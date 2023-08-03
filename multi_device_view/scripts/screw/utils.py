#!/usr/bin/env python3
# -*- coding: utf-8 -*

import numpy as np
import rospy
import skrobot
import tf
import time

from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped
from visualization_msgs.msg import Marker

def ir2ri(time=5, wait_interpolation=True):
    ri.angle_vector(robot.angle_vector(), time)
    if wait_interpolation:
        ri.wait_interpolation()

def ri2ir():
    robot.angle_vector(ri.angle_vector())

def grasp(time=0, arm='lr', range= 0.0):
    if arm == 'lr':
        rospy.sleep(time)
        ri.move_gripper('larm', range)
        ri.move_gripper('rarm', range)
    elif arm == 'l':
        rospy.sleep(time)
        ri.move_gripper('larm', range)
    elif arm == 'r':
        rospy.sleep(time)
        ri.move_gripper('rarm', range)

def drop(time=0, arm='lr'):
    if arm == 'lr':
        rospy.sleep(time)
        ri.move_gripper('larm', 1.0)
        ri.move_gripper('rarm', 1.0)
    elif arm == 'l':
        rospy.sleep(time)
        ri.move_gripper('larm', 1.0)
    elif arm == 'r':
        rospy.sleep(time)
        ri.move_gripper('rarm', 1.0)

def lookup_transform(from_tf, to_tf, stamp=rospy.Time(0), wait_time=5):
    tf_lis.waitForTransform(
        from_tf, to_tf,
        stamp, rospy.Duration(wait_time))
    return tf_lis.lookupTransform(from_tf, to_tf, stamp)


# #ここで線の傾き、末端の位置の処理をする
# class Driver():
#     def __init__(self):
#         self.msg = None
#         self.start = None
#         self.end = None
#         self.tip = None
        

#     def cb(self, msg):
#         self.msg = msg
        

# def get_driver_tip():
#     driver = Driver() 
#     s = rospy.Subscriber("/driver/line_segment_detector/debug/line_marker", Marker, driver.cb)

