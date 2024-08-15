#!/usr/bin/env python3
# -*- coding: utf-8 -*

import sys
import os
import rospkg

rospack = rospkg.RosPack()

sys.path.append(rospack.get_path("memory_insertion") + "/scripts")
from recognition import *
from demo import *

import numpy as np
from numpy import pi
import rospy
import skrobot
import tf
import time
import math

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped
from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.coordinates.quaternion import Quaternion
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords

rospy.init_node("memory_insertion")
robot = skrobot.models.PR2()
ri = PR2ROSRobotInterface(robot)

viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(robot)
viewer.show()

# tf_buffer = tf2_ros.Buffer()
# tf_listener = tf2_ros.TransformListener(tf_buffer)

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

robot.larm.move_end_pos([-0.1, 0, 0.0], "world")
robot.rarm.move_end_pos([-0.1, 0, 0.0], "world")

"""
diff_angle_rad = math.radians(5) #tmp
R_camera2 = np.array([[0,0,-1],[1,0,0],[0,-1,0]])
q_camera2_buf = skrobot.coordinates.Coordinates([0,0,0], R_camera2)
q_camera2 =  Quaternion(q_camera2_buf.quaternion)
q_lgripper = Quaternion(robot.larm_end_coords.quaternion)
q_lgripper2camera2 = q_lgripper.inverse * q_camera2
q_delta = Quaternion([math.cos(diff_angle_rad/2), 0, 0, math.sin(diff_angle_rad/2)])
q = q_lgripper2camera2 * q_delta * q_lgripper2camera2.inverse
# q.T()[0:3,0:3]
q_rarm_buf = Quaternion(robot.rarm.end_coords.quaternion)
new_coords = skrobot.coordinates.Coordinates(robot.rarm.end_coords.copy_coords().translation, (q_delta*q_rarm_buf).q)
robot.inverse_kinematics(
    new_coords,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
"""


def rotate_hand_in_cam(cam, angle_deg):
    angle_rad = math.radians(angle_deg) #tmp
    R_cam = np.array([[0,0,-1],[1,0,0],[0,-1,0]]) # todo
    q_cam_buf = skrobot.coordinates.Coordinates([0,0,0], R_cam)
    q_cam =  Quaternion(q_cam_buf.quaternion)
    q_lgripper = Quaternion(robot.larm.end_coords.copy_coords().quaternion)
    q_lgripper2cam = q_lgripper.inverse * q_cam
    q_delta = Quaternion([math.cos(angle_rad/2), 0, 0, math.sin(angle_rad/2)])
    q = q_lgripper2cam * q_delta * q_lgripper2cam.inverse
    # q.T()[0:3,0:3]
    q_larm_buf = Quaternion(robot.larm.end_coords.copy_coords().quaternion)
    new_coords = skrobot.coordinates.Coordinates(robot.larm.end_coords.copy_coords().translation, (q*q_larm_buf).q)
    robot.inverse_kinematics(
        new_coords,
        link_list=larm_link_list,
        move_target=larm_end_coords)
