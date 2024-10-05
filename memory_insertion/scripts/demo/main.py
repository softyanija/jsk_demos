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
import math

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped
from opencv_apps.msg import LineArrayStamped
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


#move-to-init()
ri.move_gripper("larm", 0.06)
ri.move_gripper("rarm", 0.06)
robot.angle_vector(ri.angle_vector())
robot.angle_vector(params.init_pose)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()
rospy.sleep(1)

#place-cam2()
robot.angle_vector(params.place_cam2_2)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

ri.move_gripper("rarm", 0.035)
rospy.sleep(1)

robot.angle_vector(params.place_cam2_3)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam2_4)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam2_5)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam2_6)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

ri.move_gripper("rarm", 0.050)
rospy.sleep(1)

robot.angle_vector(params.place_cam2_5)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam2_4)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

#move-to-init()
robot.angle_vector(ri.angle_vector())
robot.angle_vector(params.init_pose)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

#place-cam1()
robot.angle_vector(params.place_cam1_1)
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam1_2)
robot.rarm.move_end_rot(-1.4, "y")
robot.rarm.move_end_rot(-3.14, "x")
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

ri.move_gripper("rarm", 0.035)
rospy.sleep(1)

robot.angle_vector(params.place_cam1_3)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam1_4)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam1_5)
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam1_6)
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

robot.rarm.move_end_pos([0, 0, -0.012], "world")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

ri.move_gripper("rarm", 0.55)
rospy.sleep(1)

robot.angle_vector(params.place_cam1_7)
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

#move to set-cam1-tf
ri.move_gripper("rarm", 0.0)
rospy.sleep(1)

robot.rarm.move_end_rot(0.9, "y")
robot.rarm.move_end_pos([0.015, 0.04, -0.2], "world")

ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

#set-cam1-tf()
#move gripper
setter_cam1 = SetCameraTf("timer_cam1")
setter_cam1.estimate_tf()
setter_cam1.set_estimated_tf()


#move to set-cam2-tf
robot.rarm.move_end_pos([-0.01, 0.10, 0.0], "world")
robot.rarm.move_end_rot(-1.57, "x")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

#set-cam2-tf()
setter_cam2 = SetCameraTf("timer_cam2")
setter_cam2.estimate_tf()
setter_cam2.set_estimated_tf()


#move-to-init()
robot.rarm.move_end_pos([0.0, -0.08, 0.01], "world")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

robot.rarm.move_end_pos([0.0, 0.0, 0.2], "world")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

ri.move_gripper("larm", 0.039)
ri.move_gripper("rarm", 0.039)
robot.angle_vector(ri.angle_vector())

robot.angle_vector(params.init_pose)
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.rarm_down)
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)


#measure-socket-line-cam2()
# detect_lines_cam2 = DetectLinesCam2()
socket_line_cam2 = SocketLineCam2()

while True:
    rospy.loginfo("Detecting socket_line...")
    socket_line_cam2.oneshot()

    if socket_line_cam2.socket_line.lines != []:
        socket_line = socket_line_cam2[0]


#pick-memory()
robot.angle_vector(params.pick_memory_1)
robot.inverse_kinematics(
    skrobot.coordinates.Coordinates([0.764215, -0.30559, -0.831716], [0.357, 0.162, 0.032]),
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
robot.rarm.move_end_pos([0, 0, -0.1], "world")
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.pick_memory_1)
robot.inverse_kinematics(
    skrobot.coordinates.Coordinates([0.764215, -0.30559, -0.831716], [0.357, 0.162, 0.032]),
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

ri.move_gripper("larm", 0.002)
robot.angle_vector(ri.angle_vector())
rospy.sleep(1)

robot.angle_vector(params.pick_memory_1)
robot.inverse_kinematics(
    skrobot.coordinates.Coordinates([0.764215, -0.30559, -0.831716], [0.357, 0.162, 0.032]),
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
robot.rarm.move_end_pos([0, 0, -0.1], "world")
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.pick_memory_2)
robot.inverse_kinematics(
    skrobot.coordinates.Coordinates([0.764215, -0.30559, -0.831716], [0.357, 0.162, 0.032]),
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.pick_memory_3)
robot.inverse_kinematics(
    skrobot.coordinates.Coordinates([0.764215, -0.30559, -0.831716], [0.357, 0.162, 0.032]),
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.pick_memory_4)
robot.inverse_kinematics(
    skrobot.coordinates.Coordinates([0.764215, -0.30559, -0.831716], [0.357, 0.162, 0.032]),
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)


#adjust-angle-cam2()
memory_line_cam2 = MemoryLineCam2()
angle_goal_cam2 = -3

socket_angle_cam2 = math.degrees(math.atan( - (socket_line.pt2.y - socket_line.pt1.y) / (socket_line.pt2.x - socket_line.pt1.x)))

diff_angle = 100

while True:
    memory_line_cam2.oneshot()
    if (not memory_line_cam2.memory_line.lines == []):
        memory_angle_cam2 = math.degrees(memory_line_cam2.memory_line.memory_angle)
        diff_angle_cam2 = memory_angle_cam2 - socket_angle_cam2
        rospy.loginfo("diff_angle_cam2 : {}".format(diff_angle))

        if abs(diff_angle) < 3:
            rospy.loginfo("adjusted memory angle in cam2")
            break
        else:
            new_endcoords = utils.rotate_hand_in_cam(setter_cam2.estimate_tf, -angle_deg, robot.larm.end_coords)
            robot.inverse_kinematics(
                new_endcoords,
                link_list=larm_link_list,
                move_target=larm_end_coords)
            ri.angle_vector(robot.angle_vector(), 3)
            ri.wait_interpolation()
        
    else:
        rospy.loginfo("couldn't get memory line in cam2")
        continue

    rospy.sleep(5)


#adjust-angle-cam1()
memory_socket_cam1 = MemorySocketCam1()

diff_angle_tr = -3

while abs(diff_angle) < 3:
    memory_socket_cam1.oneshot()
    if (memory_socket_cam1.memory_angle is not None) and  (memory_socket_cam1.socket_angle is not None):
        diff_angle_cam1 = memory_socket_cam1.memory_angle - memory_socket_cam1.socket_angle - diff_angle_tr_cam1
        rospy.loginfo("diff_angle_cam1 : {}".format(diff_angle_cam1))
        if abs(diff_angle) < 3:
            rospy.loginfo("adjusted memory angle in cam1")
            break
        else:
            new_endcoords = utils.rotate_hand_in_cam(setter_cam2.estimate_tf, -angle_deg, robot.larm.end_coords)
            robot.inverse_kinematics(
                new_endcoords,
                link_list=larm_link_list,
                move_target=larm_end_coords)
            ri.angle_vector(robot.angle_vector(), 3)
            ri.wait_interpolation()

    else:
        rospy.loginfo("couldn't get memory or socket in cam1")
        continue

    rospy.sleep(5)


#get-cam1-pos-param()
rospy.loginfo("measuring cam1-pos-param")

while True:
    memory_socket_cam1.oneshot()
    if (not memory_socket_cam1.memory_under.poses == []):
        guide_point_cam1 = memory_socket_cam1.memory_under.poses[0]
        break

guide_point_cam1_before = guide_point_cam1.position

R_cam1 = skrobot.coordinates.Coordinates([0, 0, 0],
                                         [setter_cam1.estimated_tf.transform.rotation.w,
                                          setter_cam1.estimated_tf.transform.rotation.x,
                                          setter_cam1.estimated_tf.transform.rotation.y,
                                          setter_cam1.estimated_tf.transform.rotation.z]).rotation
reference_movement_cam1 = R_cam1.dot(np.array([-0.05,-0.05,0]))
robot.larm.move_end_pos(reference_movement_cam1, "world")
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

while True:
    if (not memory_socket_cam1.memory_under.poses == []):
        guide_point_cam1 = memory_socket_cam1.memory_under.poses[0]
        break

guide_point_cam1_after = guide_point_cam1.position

k1_x = (0.0 - 0.05) / (guide_point_cam1_after.x - guide_point_cam1_before.x)
k1_y = (0.0 - 0.05) / (guide_point_cam1_after.y - guide_point_cam1_before.y)


#get-cam2-pos-param()
rospy.loginfo("measuring cam2-pos-param")
memory_socket_cam2 = MemorySocketcam2()

while True:
    if (not memory_socket_cam2.memory_under.poses == []):
        guide_point_cam2 = memory_socket_cam2.memory_under.poses[0]
        break

guide_point_cam2_before = guide_point_cam2.position

R_cam2 = skrobot.coordinates.Coordinates([0, 0, 0],
                                         [setter_cam2.estimated_tf.transform.rotation.w,
                                          setter_cam2.estimated_tf.transform.rotation.x,
                                          setter_cam2.estimated_tf.transform.rotation.y,
                                          setter_cam2.estimated_tf.transform.rotation.z]).rotation
reference_movement_cam2 = R_cam2.dot(np.array([-0.05,-0.05,0]))
robot.larm.move_end_pos(reference_movement_cam2, "world")
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

while True:
    if (not memory_socket_cam2.memory_under.poses == []):
        guide_point_cam2 = memory_socket_cam2.memory_under.poses[0]
        break

guide_point_cam2_after = guide_point_cam2.position

k2_x = (0.0 - 0.05) / (guide_point_cam2_after.x - guide_point_cam2_before.x)
k2_y = (0.0 - 0.05) / (guide_point_cam2_after.y - guide_point_cam2_before.y)


#adjust-pos-cam1()
rospy.loginfo("adjusting pos-cam1")
x_cam1_tolerance = 2
y_cam1_tolerance = 4

while True:
    memory_socket_cam1.oneshot()
    if (not memory_socket_cam1.memory_under_pose == []) and (memory_socket_cam1.socket_target_point is not None):
        memory_guide_point_cam1 = memory_socket_cam1.memory_under_pose.position
        socket_target_point_cam1 = memory_socket_cam1.socket_target_point
        x_diff_cam1 = memory_guide_point_cam1.x - socket_target_point_cam1[0] 
        y_diff_cam1 = memory_guide_point_cam1.y - socket_target_point_cam1[1]
        rospy.loginfo("x_diff_cam1: {}, y_diff_cam1: {}".format(x_diff_cam1, y_diff_cam1))

        if (abs(x_diff_cam1) < x_cam1_tolerance) and (abs(y_diff_cam1) < y_cam1_tolerance):
            rospy.loginfo("adjust-pos-cam1 finished")
            break
        else:
            rospy.loginfo("moving hand to adjust-pos-cam1")
            delta_px = np.array([k1_x * x_diff_cam1, k1_y * y_diff_cam1, 0])
            delta_world = np.dot(R_cam1, delta_px)
            robot.larm.move_end_pos(delta_world, "world")
            ri.angle_vector(robot.angle_vector(), 2)
            ri.wait_interpolation()
            rospy.sleep(2)


#adjust-pos-cam2()
rospy.loginfo("adjusting pos-cam2")

while True:
    memory_socket_cam2.oneshot()
    if (not memory_socket_cam2.memory_guide_point_b.poses == []) and (memory_socket_cam2.socket_target_point_b is not None):
        memory_guide_point_cam2 = memory_socket_cam2.memory_guide_point_b.poses[0].position
        socket_target_point_cam2 = memory_socket_cam2.socket_target_point
        x_diff_cam2 = memory_guide_point_cam2.x - socket_target_point_cam2[0] 
        y_diff_cam2 = memory_guide_point_cam2.y - socket_target_point_cam2[1]
        rospy.loginfo("x_diff_cam2: {}, y_diff_cam2: {}".format(x_diff_cam2, y_diff_cam2))

        if (abs(x_diff_cam2) < x_cam2_tolerance) and (abs(y_diff_cam2) < y_cam2_tolerance):
            rospy.loginfo("adjust-pos-cam2 finished")
            break
        else:
            rospy.loginfo("moving hand to adjust-pos-cam2")
            delta_px = np.array([k2_x * x_diff_cam2, k2_y * y_diff_cam2, 0])
            delta_world = np.dot(R_cam2, delta_px)
            robot.larm.move_end_pos(delta_world, "world")
            ri.angle_vector(robot.angle_vector(), 2)
            ri.wait_interpolation()
            rospy.sleep(2)


#adjust-pos-cam1()
rospy.loginfo("adjusting pos-cam1 again")
while True:
    memor_socket_cam1.oneshot()
    if (not memory_socket_cam1.memory_under.pose == []) and (memory_socket_cam1.socket_target_point is not None):
        memory_guide_point_cam1 = memory_socket_cam1.memory_under.pose.position
        socket_target_point_cam1 = memory_socket_cam1.socket_target_point
        x_diff_cam1 = memory_guide_point_cam1.x - socket_target_point_cam1[0] 
        y_diff_cam1 = memory_guide_point_cam1.y - socket_target_point_cam1[1]
        rospy.loginfo("x_diff_cam1: {}, y_diff_cam1: {}".format(x_diff_cam1, y_diff_cam1))

        if (abs(x_diff_cam1) < x_cam1_tolerance) and (abs(y_diff_cam1) < y_cam1_tolerance):
            rospy.loginfo("adjust-pos-cam1 finished")
            break
        else:
            rospy.loginfo("moving hand to adjust-pos-cam1")
            delta_px = np.array([k1_x * x_diff_cam1, k1_y * y_diff_cam1, 0])
            delta_world = np.dot(R_cam1, delta_px)
            robot.larm.move_end_pos(delta_world, "world")
            ri.angle_vector(robot.angle_vector(), 2)
            ri.wait_interpolation()
            rospy.sleep(2)


#set-memory()
robot.larm.move_end_pos([0, 0.003, -0.006], "world")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(2)

robot.larm.move_end_pos([0.002, 0, 0], "world")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(2)

robot.larm.move_end_pos([-0.002, 0, 0], "world")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(2)

robot.larm.move_end_pos([0, -0.002, 0], "world")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(2)

robot.larm.move_end_pos([0, 0, -0.005], "world")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(2)

robot.larm.move_end_pos([0, -0.003, 0], "world")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(2)

robot.angle_vector(ri.angle_vector())

ri.move_gripper("larm", 0.015)
rospy.sleep(1)

#judge memory_state

while True:
    memory_socket_cam2.oneshot()
    if memory_socket_cam2.socket_size.width * memory_socket_cam2.socket_size.height < 100:
        rospy.loginfo("memory is front side")
        memory_state = "front"
    else:
        memory_guide_point_cam2 = memory_socket_cam2.memory_guide_point_b.poses[0].position
        diff_x_cam2 = memory_guide_point_cam2.x - memory_socket_cam2.socket_center.x
        diff_y_cam2 = memory_guide_point_cam2.y - memory_socket_cam2.socket_center.y

        if diff_y_cam2 > 1:
            rospy.loginfo("memory is set")
            memory_state = "set"
        elif (diff_y_cam2 > -4) and (diff_y_cam2 < 1):
            rospy.loginfo("memory is rear")
            memory_state = "rear"
        else:
            rospy.loginfo("memory is on_socket")
            memory_state = "on_socket"

    if memory_state = "set":
        rospy.loginfo("success memory setting!!")
        break;

    elif(memory_state == "rear"):
        robot.larm.move_end_pos([-0.002, 0, 0.007], "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(1)

        robot.larm.move_end_pos([0, 0, -0.007], "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(1)

        ri.move_gripper("larm", 0.004)

        robot.larm.move_end_pos([0, -0.001, 0.007], "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(1)

        robot.larm.move_end_pos([0.005, 0, 0], "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(1)

        ri.move_gripper("larm", 0.015)
        rospy.sleep(1)

    elif(memory_state == "front"):
        robot.larm.move_end_pos([0.002, 0, 0.007], "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(1)

        robot.larm.move_end_pos([0, 0, -0.007], "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(1)

        ri.move_gripper("larm", 0.004)

        robot.larm.move_end_pos([0, -0.002, 0.007], "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(1)

        robot.larm.move_end_pos([-0.005, 0, 0], "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(1)

        ri.move_gripper("larm", 0.015)
        rospy.sleep(1)

    elif(memory_state == "on_socket"):
        ri.move_gripper("larm", 0.010)
        robot.larm.move_end_pos([-0.003, 0, 0], "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(1)

        robot.larm.move_end_pos([0.006, 0, 0], "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(1)

#push-memory()
size_limit_cam1 = 30
size_limit_cam2 = 30

robot.angle_vector(ri.angle_vector())

robot.larm.move_end_pos([0, 0.015, 0.014], "world")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

wrist_angle = robot.l_wrist_roll_joint.joint_angle()
robot.l_wrist_roll_joint.joint_angle(wrist_angle + 1.57)
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()

robot.larm.move_end_rot(0.785, "x")
robot.larm.move_end_rot(-0.0175, "z")
robot.larm.move_end_rot(0.0175, "y")
ri.angle_vector(robot.angle_vector(), 2)
ri.wait_interpolation()
rospy.sleep(1)

while True:
    memory_socket_cam2.oneshot()
    if (memory_socket_cam2.socket_target_point_b is not None):
        break;

sokcet_cam2_x = memory_socket_cam2.socket_center.x
sokcet_cam2_y = memory_socket_cam2.socket_center.y
goal_x = -30
goal_y = -105
hand_goal_x = sokcet_cam2_x + goal_x
hand_goal_y = sokcet_cam2_y + goal_y
x_tr = 3
y_tr = 5
k2_x = 0.25
k2_y = 0.25

apriltag_hand_pos = ApriltagHandPos()

while True:
    apriltag_hand_pos.oneshot()

    if apriltag_hand_pos.hand_pos is not None:
        x_diff_cam2 = apriltag_hand_pos.hand_pos[0] - hand_goal_x
        y_diff_cam2 = apriltag_hand_pos.hand_pos[1] - hand_goal_y

    if abs(diff_x) < x_tr and abs(diff_y) < y_tr:
        rospy.loginfo("moving hand to push position")
        delta_cam = np.array([k2_x * x_diff_cam2, k2_y * y_diff_cam2, 0])
        delta_world = np.dot(R_cam2, delta_cam)
        robot.larm.move_end_pos(delta_world, "world")
        ri.angle_vector(robot.angle_vector(), 2)
        ri.wait_interpolation()
        rospy.sleep(2)
    else:
        rospy.loginfo("finished moving hand to push position")
        return


lock_th_cam1 = 30
lock_th_cam2 = 30

check_lever_cam2 =CheckLeverCam2()

while True:
    robot.larm.move_end_rot(math.radians(8), "z")
    ri.angle_vector(robot.angle_vector(), 2)
    ri.wait_interpolation()
    rospy.sleep(1)

    robot.torso_lift_joint.joint_angle(robot.torso_lift_joint.joint_angle() - 30)
    ri.angle_vector(robot.angle_vector(), 2)
    ri.wait_interpolation()
    rospy.sleep(1)

    robot.torso_lift_joint.joint_angle(robot.torso_lift_joint.joint_angle() + 20)
    ri.angle_vector(robot.angle_vector(), 2)
    ri.wait_interpolation()
    rospy.sleep(1)

    robot.larm.move_end_rot(math.radians(-8), "z")
    ri.angle_vector(robot.angle_vector(), 2)
    ri.wait_interpolation()
    rospy.sleep(1)

    CheckLeverCam2.oneshot()
    if check_lever_cam2.size > lock_th_cam2:
        rospy.loginfo("memory of side cam2 isn't locked")
    else:
        rospy.loginfo("memory of side cam2 is locked")
        return

robot.larm.move_end_pos([0, -65, 0], "world")
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)

# check_lever_cam1 =CheckLeverCam1()
while True:
    robot.larm.move_end_rot(math.radians(-7), "z")
    ri.angle_vector(robot.angle_vector(), 2)
    ri.wait_interpolation()
    rospy.sleep(1)

    robot.torso_lift_joint.joint_angle(robot.torso_lift_joint.joint_angle() - 32)
    ri.angle_vector(robot.angle_vector(), 2)
    ri.wait_interpolation()
    rospy.sleep(1)

    robot.torso_lift_joint.joint_angle(robot.torso_lift_joint.joint_angle() 20)
    ri.angle_vector(robot.angle_vector(), 2)
    ri.wait_interpolation()
    rospy.sleep(1)

    robot.larm.move_end_rot(math.radians(7), "z")
    ri.angle_vector(robot.angle_vector(), 2)
    ri.wait_interpolation()
    rospy.sleep(1)

    # check_lever_cam1.oneshot()
    check_lever_cam1 = 0
    if check_lever_cam1.size > lock_th_cam1:
        rospy.loginfo("memory of side cam1 isn't locked")
    else:
        rospy.loginfo("memory of side cam1 is locked")
        return

robot.larm.move_end_pos([0, 30, 100], "world")
ri.angle_vector(robot.angle_vector(), 3)
ri.wait_interpolation()
rospy.sleep(1)
