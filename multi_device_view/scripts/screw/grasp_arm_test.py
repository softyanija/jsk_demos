# -*- coding: utf-8 -*

import numpy as np
import rospy
import skrobot
import tf
import tf2_ros
import time
import math

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped
from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords
from jsk_recognition_msgs.msg import BoundingBoxArray

from utils import *
from driver_tip import Driver
from screw_hanged import ScrewHanged
from set_d405_tf import Set_d405_tf
from wrench import Wrench

rospy.init_node("inducdtion_screw")
robot = skrobot.models.PR2()
ri = PR2ROSRobotInterface(robot)

viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(robot)
viewer.show()

# driver = Driver()
# driver_subscriber = rospy.Subscriber("/driver/line_segment_detector/debug/line_marker", Marker, driver.cb)
screw_hanged = ScrewHanged()
screw_hanged_subscriber = rospy.Subscriber("/screw/euclidean_clustering_decomposer/boxes", BoundingBoxArray, screw_hanged.cb)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
# self.tool_frame = tf_buffer.lookup_transform("camera_color_optical_frame", "r_gripper_tool_frame", rospy.Time(), rospy.Duration(3))

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

# move to near screw
induction_init_angle_vector = np.array([-1.5704848e+00, 4.5554123e+01, 4.2743446e+01, -1.5701544e+00,
                                        -3.7792927e+01, -3.9414707e+01, -7.8521786e+00, -1.4776426e+01,
                                        -2.9745804e+01, 1.0994394e+01, -6.0272266e+01, -4.9123852e+01,
                                        2.9614183e-01, 0.0000000e+00, 5.1312681e-02, 1.2962595e+00,
                                        1.8070649e-02, -2.4816328e-01, 1.5658520e-01, -1.6714991e+00,
                                        -1.6782597e+00, -1.2354465e+00, -1.4277110e+00, -1.1684082e+01,
                                        0.0000000e+00, 0.0000000e+00, 7.2283611e-02, 7.2283611e-02,
                                        7.2283611e-02, 7.2283611e-02, 1.2777434e-02, 4.7342056e-01,
                                        2.9929683e-01, 1.2520109e+00, 2.0709257e+00, -1.5155778e+00,
                                        -1.5172944e+00, -9.6642292e-01, 0.0000000e+00, 0.0000000e+00,
                                        2.2267133e-01, 2.2267133e-01, 2.2267133e-01, 2.2267133e-01,
                                        3.9666355e-02], dtype="float32")
robot.angle_vector(ri.angle_vector())
robot.angle_vector(induction_init_angle_vector)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()
rospy.sleep(1)

ri.move_gripper("rarm", 0.0118)

# set d405 tf
setter = Set_d405_tf()
setter.estimate_tf()
setter.set_estimated_tf()

# grasp KXR by PR2 left hand
arm_grasp_pre_tf = tf_buffer.lookup_transform("base_footprint", "arm_grasp_pre_frame", rospy.Time(), rospy.Duration(5))
arm_grasp_tf = tf_buffer.lookup_transform("base_footprint", "arm_grasp_frame", rospy.Time(), rospy.Duration(5))

arm_grasp_pre = skrobot.coordinates.Coordinates([arm_grasp_pre_tf.transform.translation.x,
                                                 arm_grasp_pre_tf.transform.translation.y,
                                                 arm_grasp_pre_tf.transform.translation.z],
                                                [arm_grasp_pre_tf.transform.rotation.w,
                                                 arm_grasp_pre_tf.transform.rotation.x,
                                                 arm_grasp_pre_tf.transform.rotation.y,
                                                 arm_grasp_pre_tf.transform.rotation.z])
arm_grasp = skrobot.coordinates.Coordinates([arm_grasp_tf.transform.translation.x,
                                             arm_grasp_tf.transform.translation.y,
                                             arm_grasp_tf.transform.translation.z],
                                            [arm_grasp_tf.transform.rotation.w,
                                             arm_grasp_tf.transform.rotation.x,
                                             arm_grasp_tf.transform.rotation.y,
                                             arm_grasp_tf.transform.rotation.z])

robot.inverse_kinematics(
    arm_grasp_pre,
    link_list=larm_link_list,
    move_target=larm_end_coords)
ri.angle_vector(robot.angle_vector(), 5)
ri.wait_interpolation()

robot.inverse_kinematics(
    arm_grasp,
    link_list=larm_link_list,
    move_target=larm_end_coords)
ri.angle_vector(robot.angle_vector(), 5)
ri.wait_interpolation()
