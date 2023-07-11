#!/usr/bin/env python3
# -*- coding: utf-8 -*

import numpy as np
import rospy
import skrobot
import tf
import time

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped
from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords

rospy.init_node("screw_tighting")
robot = skrobot.models.PR2()
ri = PR2ROSRobotInterface(robot)
rarm_link_list = [
    r.r_shoulder_pan_link,
    r.r_shoulder_lift_link,
    r.r_upper_arm_roll_link,
    r.r_elbow_flex_link,
    r.r_forearm_roll_link, 
    r.r_wrist_flex_link,
    r.r_wrist_roll_link]

viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(robot)
viewer.show()

#demo_init()
#place_camera()
#cailbration_caemra()
#hold_driver()
#pick_screw()
#induction_screw()
#insert_screw()
#turn_screw()
#return_driver()

