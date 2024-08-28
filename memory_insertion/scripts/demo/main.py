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
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()
rospy.sleep(1)

ri.move_gripper("rarm", 0.035) #TODO: adjust
rospy.sleep(1)

robot.angle_vector(params.place_cam2_3)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam2_4)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam2_5)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam2_6)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()
rospy.sleep(1)

ri.move_gripper("rarm", 0.050)
rospy.sleep(1)

robot.angle_vector(params.place_cam2_5)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()
rospy.sleep(1)

robot.angle_vector(params.place_cam2_4)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()
rospy.sleep(1)

#move-to-init()


#place-cam1()

#set cam1 tf
setter_cam1 = SetCameraTf("timer_cam1")
setter.estimate_tf()
setter.set_estimated_tf()

#set cam2 tf
setter_cam2 = SetCameraTf("timer_cam2")
setter.estimate_tf()
setter.set_estimated_tf()

#move-to-init()
ri.move_gripper("larm", 0.039)
robot.angle_vector(ri.angle_vector())

robot.angle_vector(induction_init_angle_vector)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()
rospy.sleep(1)

#(send *pr2* :angle-vector *rarm-down*)
#(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
#(send *ri* :wait-interpolation)

#measure-socket-line-cam2()
socket_line_cam2 = SocketLineCam2()

while True:
    if socket_line_cam2.socket_line.lines != []:
        socket_line = socket_line_cam2.socket_line.lines[0]
        angle = math.degrees(math.atan( - (socket_line.pt2.y - socket_line.pt1.y) / (socket_line.pt2.x - socket_line.pt1.x)))
        
    
    


#pick-memory()
#adjust-angle-cam2()

#adjust-angle-cam1()

memory_socket_cam1 = MemorySocketCam1()

diff_angle_tr = 3

while abs(diff_angle) < 3:
    if (memory_socket_cam1.memory_angle is not None) and  (memory_socket_cam1.socket_angle is not None):
        diff_angle = memory_socket_cam1.memory_angle - memory_socket_cam1.socket_angle
    else:
        rospy.loginfo("couldn't get memory or socket in cam1")
        continue
    # calc angle here and rotation
    robot.angle_vector(ri.angle_vector())
    # rotation

    ri.angle_vector(robot.angle_vector())
    ri.wait_for_interpolation()
    rospy.sleep(5)


#get-cam1-pos-param()
rospy.loginfo("measuring cam1-pos-param")

#adjust-pos-cam1()
rospy.loginfo("adjusting pos-cam1")

#get-cam2-pos-param()
rospy.loginfo("measuring cam2-pos-param")


#adjust-pos-cam2()
rospy.loginfo("adjusting pos-cam2")



#get-cam2-pos-param()
rospy.loginfo("measuring cam2-pos-param again")

#adjust-pos-cam1()
rospy.loginfo("adjusting pos-cam1 again")

#set-memory()


#push_memory()


'''
(ros::roseus "third_eye")
(move-to-init)

(place-cam2)
(move-to-init)
(place-cam1)
(move-to-init)
(send *pr2* :angle-vector *rarm-down*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(measure-socket-line-cam2)
(pick-memory)
(adjust-angle-cam2)
(adjust-angle-cam1)

(get-cam1-pos-param)
(adjust-pos-cam1)

(get-cam2-pos-param)
(adjust-pos-cam2)

(adjust-pos-cam1)

(set-memory)

(push_memory)
'''
