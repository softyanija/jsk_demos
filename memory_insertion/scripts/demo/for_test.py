#!/usr/bin/env python3
# -*- coding: utf-8 -*

import sys
import os
import rospkg

rospack = rospkg.RosPack()

sys.path.append(rospack.get_path("memory_insertion") + "/scripts")
from recognition import MemorySocketCam1,MemorySocketCam2,MemoryEdgeCam2,MemoryLineCam2,SocketLineCam2,DetectLinesCam2,CheckLeverCam1,CheckLeverCam2,ApriltagHandPos,SetCameraTf
from demo import *

import numpy as np
import rospy
import skrobot
import tf
import tf2_ros
import time

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped
from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords

rospy.init_node("memory_insertion")
robot = skrobot.models.PR2()
ri = PR2ROSRobotInterface(robot)

viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(robot)
viewer.show()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

larm_end_coords = skrobot.coordinates.CascadedCoords(parent=robot.l_gripper_tool_frame, name="larm_end_coords")
larm_move_target = larm_end_coords
larm_link_list = [
    robot.l_shoulder_pan_link,
    robot.l_shoulder_lift_link,
    robot.l_upper_arm_roll_link,
    robot.l_elbow_flex_link,
    robot.l_forearm_roll_link,
    robot.l_wrist_flex_link,
    robot.l_wrist_roll_link]

rarm_end_coords = skrobot.coordinates.CascadedCoords(parent=robot.r_gripper_tool_frame, name="rarm_end_coords")
rarm_move_target = rarm_end_coords
rarm_link_list = [
    robot.r_shoulder_pan_link,
    robot.r_shoulder_lift_link,
    robot.r_upper_arm_roll_link,
    robot.r_elbow_flex_link,
    robot.r_forearm_roll_link,
    robot.r_wrist_flex_link,
    robot.r_wrist_roll_link]
