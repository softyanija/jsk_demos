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

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped, TransformStamped
from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.coordinates.quaternion import Quaternion
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords


def rotate_hand_in_cam(camera_tf, angle_deg, end_coords):
    angle_rad = math.radians(angle_deg)
    R_cam = skrobot.coordinates.Coordinates([0, 0, 0],
                                            [camera_tf.transform.rotation.w,
                                             camera_tf.transform.rotation.x,
                                             camera_tf.transform.rotation.y,
                                             camera_tf.transform.rotation.z]).rotation
    q_cam = Quaternion(camera_tf.transform.rotation.x,
                       camera_tf.transform.rotation.y,
                       camera_tf.transform.rotation.z,
                       camera_tf.transform.rotation.w)
    q_lgripper = Quaternion(end_coords.copy_coords().quaternion)
    q_lgripper2cam = q_lgripper.inverse * q_cam
    q_delta = Quaternion([math.cos(angle_rad/2), 0, 0, math.sin(angle_rad/2)])
    q = q_lgripper2cam * q_delta * q_lgripper2cam.inverse
    q_larm_buf = Quaternion(end_coords.copy_coords().quaternion)
    new_coords = skrobot.coordinates.Coordinates(end_coords.copy_coords().translation, (q*q_larm_buf).q)

    return new_coords


def move_hand_in_cam(camera_tf, translation=np.array([0.05, 0, 0]), k_x, k_y):    
    R_cam = np.array([[0,0,-1],[1,0,0],[0,-1,0]]) # todo
    q_cam_buf = skrobot.coordinates.Coordinates([0,0,0], R_cam)
    q_cam =  Quaternion(q_cam_buf.quaternion)
    translation_quat = Quaternion(np.array([0, *translation]))
    translation_robot = (q_cam * translation_quat * q_cam.inverse).xyz
    robot.larm.move_end_pos(translation_robot, "world")

    return translation_robot



