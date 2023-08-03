#!/usr/bin/env python3
# -*- coding: utf-8 -*

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

from utils import *
from driver_tip import Driver
from screw_hanged import ScrewHanged
from set_d405_tf import Set_d405_tf
from wrench import Wrench

rospy.init_node("screw_tighting")
robot = skrobot.models.PR2()
ri = PR2ROSRobotInterface(robot)
# rarm_link_list = [
#     r.r_shoulder_pan_link,
#     r.r_shoulder_lift_link,
#     r.r_upper_arm_roll_link,
#     r.r_elbow_flex_link,
#     r.r_forearm_roll_link, 
#     r.r_wrist_flex_link,
#     r.r_wrist_roll_link]

viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(robot)
viewer.show()

driver = Driver()
driver_subscriber = rospy.Subscriber("/driver/line_segment_detector/debug/line_marker", Marker, driver.cb)
screw_hanged = ScrewHanged()
screw_hanged_subscriber = rospy.Subscriber("/screw/line_segment_detector/debug/line_marker", Marker, screw_hanged.cb)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

    #move to near screw
induction_init_angle_vector = np.array([-1.5704848e+00,  4.5554123e+01,  4.2743446e+01, -1.5701544e+00,
                                            -3.7792927e+01, -3.9414707e+01, -7.8521786e+00, -1.4776426e+01,
                                            -2.9745804e+01,  1.0994394e+01, -6.0272266e+01, -4.9123852e+01,
                                            2.9614183e-01,  0.0000000e+00,  5.1312681e-02,  1.2962595e+00,
                                            1.8070649e-02, -2.4816328e-01,  1.5658520e-01, -1.6714991e+00,
                                            -1.6782597e+00, -1.2354465e+00, -1.4277110e+00, -1.1684082e+01,
                                            0.0000000e+00,  0.0000000e+00,  7.2283611e-02,  7.2283611e-02,
                                            7.2283611e-02,  7.2283611e-02,  1.2777434e-02,  4.7342056e-01,
                                            2.9929683e-01,  1.2520109e+00,  2.0709257e+00, -1.5155778e+00,
                                            -1.5172944e+00, -9.6642292e-01,  0.0000000e+00,  0.0000000e+00,
                                            2.2267133e-01,  2.2267133e-01,  2.2267133e-01,  2.2267133e-01,
                                            3.9666355e-02], dtype="float32")
robot.angle_vector(ri.angle_vector())
robot.angle_vector(induction_init_angle_vector)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()

#set d405 tf
setter = Set_d405_tf()
setter.estimate_tf()
setter.set_estimated_tf()
