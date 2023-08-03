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

from utils import *
from driver_tip import Driver
from screw_hanged import ScrewHanged
from set_d405_tf import Set_d405_tf
from wrench import Wrench

#def hoge

rospy.init_node("inducdtion_screw")
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
#self.tool_frame = tf_buffer.lookup_transform("camera_color_optical_frame", "r_gripper_tool_frame", rospy.Time(), rospy.Duration(3))

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
rospy.sleep(1)

#set d405 tf
setter = Set_d405_tf()
setter.estimate_tf()
setter.set_estimated_tf()

#induction_near_screw

#
    
#姿勢合わせ 一旦後回し
#while

#adjust pos

diff_range_mm = 100
while diff_range_mm > 1:
    driver.pub_tip_frame_tf()
    screw_hanged.pub_tip_frame_tf()

    screw_hole_induced_pre_tf = tf_buffer.lookup_transform("base_footprint", "screw_hole_induced_pre", rospy.Time(), rospy.Duration(5))
    screw_induction_tf = tf_buffer.lookup_transform("base_footprint", "screw_induction_frame", rospy.Time(), rospy.Duration(5))

    pos_diff = [screw_hole_induced_pre_tf.transform.translation.x - screw_induction_tf.transform.translation.x,
                screw_hole_induced_pre_tf.transform.translation.y - screw_induction_tf.transform.translation.y,
                screw_hole_induced_pre_tf.transform.translation.z - screw_induction_tf.transform.translation.z
]
    print("pos_diff: " + str(pos_diff))

    robot.angle_vector(ri.angle_vector())
    robot.rarm.move_end_pos(pos_diff, "world")

    ri.angle_vector(robot.angle_vector(), 3)
    ri.wait_interpolation()
    rospy.sleep(2)

    diff_range_mm = math.sqrt((pos_diff[0]**2 +  pos_diff[1]**2 +  pos_diff[2]**2)) * 10**3


# screw_hole_induced_pre_tf = tf_buffer.lookup_transform("base_footprint", "screw_hole_induced_pre", rospy.Time(), rospy.Duration(5))
# screw_hole_induced_tf = tf_buffer.lookup_transform("base_footprint", "screw_hole_induced", rospy.Time(), rospy.Duration(5))
# screw_hole_induced_diff = [screw_hole_induced_tf.transform.translation.x - screw_hole_induced_pre_tf.transform.translation.x,
#                           screw_hole_induced_tf.transform.translation.y - screw_hole_induced_pre_tf.transform.translation.y,
#                           screw_hole_induced_tf.transform.translation.z - screw_hole_induced_pre_tf.transform.translation.z]

# robot.angle_vector(ri.angle_vector())
# robot.rarm.move_end_pos(screw_hole_induced_diff, "world")

# ri.angle_vector(robot.angle_vector(), 3)
# ri.wait_interpolation()

# wrench = Wrench()
# wrench_subscriber = rospy.Subscriber("/right_endeffector/wrench", WrenchStamped, wrench.cb)

# times_limit = 4
# counter_force_limit = -0.5
# insert_range = [0, 0, -0.002]
# rospy.sleep(1)
# for i in range(times_limit):
#     if wrench.fx < counter_force_limit:
#         break
#     robot.angle_vector(ri.angle_vector())
#     robot.rarm.move_end_pos(insert_range, "world")
#     ri.angle_vector(robot.angle_vector(), 3)
#     ri.wait_interpolation()

# robot.angle_vector(ri.angle_vector())
# wrist_angle = robot.l_wrist_roll_joint.joint_angle()
# robot.l_wrist_roll_joint.joint_angle(wrist_angle + 3.14)
# ri.angle_vector(robot.angle_vector(), 2)
# ri.wait_interpolation()

# for i in range(16):
#     print("loop" + str(i))
#     robot.angle_vector(ri.angle_vector())
#     wrist_angle = robot.r_wrist_roll_joint.joint_angle()
#     robot.r_wrist_roll_joint.joint_angle(wrist_angle + 3.14)
#     ri.angle_vector(robot.angle_vector(), 2)
#     ri.wait_interpolation()
#     if wrench.fx > counter_force_limit:
#         robot.rarm.move_end_pos(insert_range, "world")
#         ri.angle_vector(robot.angle_vector(), 3)
#         ri.wait_interpolation()

# #しまっているかの判定ほしい

# robot.angle_vector(ri.angle_vector())
# robot.rarm.move_end_pos([0, 0, 0.006], "world")
# ri.angle_vector(robot.angle_vector(), 3)
# ri.wait_interpolation()
