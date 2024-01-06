# -*- coding: utf-8 -*

import numpy as np
import rospy
import skrobot
import tf
import tf2_ros
import time
import math
import pdb

from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords
from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from dynamic_tf_publisher.srv import SetDynamicTF
from param import *


def get_tag_coordinates(camera_position, tag_name):
    # camera_position:{rarm, larm, head}
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tag_frame = camera_position + "_" + tag_name
    b2t = None

    try:
        b2t = tf_buffer.lookup_transform("base_link", tag_frame, rospy.Time(), rospy.Duration(3))
        base_to_tag = skrobot.coordinates.Coordinates([b2t.transform.translation.x,
                                                       b2t.transform.translation.y,
                                                       b2t.transform.translation.z],
                                                      [b2t.transform.rotation.w,
                                                       b2t.transform.rotation.x,
                                                       b2t.transform.rotation.y,
                                                       b2t.transform.rotation.z]
            )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to get transform")
        base_to_tag = None
        # pdb.set_trace()

    return base_to_tag

def get_tf_coordinates(tf_name):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    b2t = None
    try:
        b2t = tf_buffer.lookup_transform("base_link", tf_name, rospy.Time(), rospy.Duration(3))
        base_to_tf = skrobot.coordinates.Coordinates([b2t.transform.translation.x,
                                                       b2t.transform.translation.y,
                                                       b2t.transform.translation.z],
                                                      [b2t.transform.rotation.w,
                                                       b2t.transform.rotation.x,
                                                       b2t.transform.rotation.y,
                                                       b2t.transform.rotation.z]
            )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to get transform")
        base_to_tf = None
        # pdb.set_trace()

    return base_to_tf

def set_tf(coordinates, frame_name):
    tf = TransformStamped()
    tf.header.frame_id = "base_link"
    tf.child_frame_id = frame_name
    tf.transform.translation.x = coordinates.translation[0]
    tf.transform.translation.y = coordinates.translation[1]
    tf.transform.translation.z = coordinates.translation[2]
    tf.transform.rotation.x = coordinates.quaternion[1]
    tf.transform.rotation.y = coordinates.quaternion[2]
    tf.transform.rotation.z = coordinates.quaternion[3]
    tf.transform.rotation.w = coordinates.quaternion[0]
    
    rospy.wait_for_service("/set_dynamic_tf")
    try:
        client = rospy.ServiceProxy("/set_dynamic_tf", SetDynamicTF)
        res = client(100, tf)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



rospy.init_node("attatch_upper_arm")
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


robot.angle_vector(ri.angle_vector())
robot.angle_vector(init_angle_vector)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()


robot.rarm.angle_vector(search_arm_rarm_vector)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()

pos_counter = 0
tag_recognition_counter = 0
move_radius = 28
tag_is_found = False

try:
    for i in range(4):
        for j in range(3):
            j += 1
            tag_coordinates = get_tag_coordinates("rarm", "kxr_arm")
            if tag_coordinates is not None:
                rospy.loginfo("Found arm tag")
                tag_is_found = True
                break
            rospy.loginfo("Coudn't find arm tag : {} times".format(j))

        if tag_is_found:
            break
        else:
            #move arm to find tag
            rospy.loginfo("move to pos{}".format(i+1))
            theta = math.pi/4 + i*math.pi/2
            robot.rarm.move_end_pos([move_radius*math.cos(theta), move_radius*math.sin(theta), 0], "world")
            ri.angle_vector(robot.angle_vector(), 4)
            ri.wait_interpolation()
            i += 1

            if i == 4:
                rospy.logwarn("Coudn't find arm tag in 4 pos, loop again")
                i = 0
except KeyboardInterrupt as e:
    print(e)

#publish tf from rarm_kxr(tag_coordinates) to kxr_arm
set_tf(tag_coordinates, "kxr_arm")
servo_gear_coordinates = tag_coordinates.transform(kxr_arm_to_servo_gear)
set_tf(servo_gear_coordinates, "servo_gear")

# #grasp arm
# grasp_pos = tag_coordinates.copy_worldcoords().transform(tag_to_grasp_pos)


# ri.move_gripper("rarm", 0.06)
# robot.inverse_kinematics(
#     grasp_pos,
#     link_list=rarm_link_list,
#     move_target=rarm_end_coords)
# robot.rarm.move_end_pos([-0.04, 0, 0], "local")
# ri.angle_vector(robot.angle_vector(), 4)
# ri.wait_interpolation()
# rospy.sleep(1)


# # grasp kxr arm
# ri.angle_vector(robot.angle_vector(), 4)
# ri.wait_interpolation()
# rospy.sleep(1)
# ri.move_gripper("rarm", 0.0118)


# # search stored_modules 
# robot.larm.angle_vector(search_stored_module_larm_vector)
# ri.angle_vector(robot.angle_vector(), 4)
# ri.wait_interpolation()


# module_0_coordinates = None
# while module_0_coordinates is None:
#     rospy.loginfo("getting module_0 pos")
#     module_0_coordinates = get_tag_coordinates("larm", "module_0")

# module_0_grasp_pos = module_0_coordinates.copy_worldcoords().transform(module_tag_to_grasp_pos)

# robot.larm.angle_vector(grasp_module_neutral_larm_vector)
# ri.angle_vector(robot.angle_vector(), 4)
# ri.wait_interpolation()


# robot.inverse_kinematics(
#     module_0_grasp_pos,
#     link_list=larm_link_list,
#     move_target=larm_end_coords)
# ri.angle_vector(robot.angle_vector(), 4)
# ri.wait_interpolation()
# rospy.sleep(1)

# robot.larm.move_end_pos([0.054, 0, 0], "local")
# ri.angle_vector(robot.angle_vector(), 3)
# ri.wait_interpolation()

# # grasp module
# ri.move_gripper("larm", 0.019)

# robot.larm.move_end_pos([-0.100, 0, 0], "local")
# ri.angle_vector(robot.angle_vector(), 4)
# ri.wait_interpolation()

# # calc camera position
# servo_gear_coordinates = None
# while servo_gear_coordinates is None:
#     rospy.loginfo("getting servo_gear pos")
#     module_0_coordinates = get_tf_coordinates("servo_gear")

# table_top_z = 0.73 
# module_0_pos = module_0_coordinates.transform(servo_gear_to_module_0)
# module_1_pos = module_1_coordinates.transform(servo_gear_to_module_1)    

# module_0_pos.translation([module_0_pos.translation[0], module_0_pos.translation[1], table_top_z])
# module_1_pos.translation([module_1_pos.translation[0], module_1_pos.translation[1], table_top_z])


# ri.move_gripper("larm", 0.06)
# robot.angle_vector(ri.angle_vector())
# robot.angle_vector(induction_init_angle_vector)
# ri.angle_vector(robot.angle_vector(), 4)
# ri.wait_interpolation()
# rospy.sleep(1)

# ri.move_gripper("rarm", 0.0118)

# robot.inverse_kinematics(
#     arm_grasp_pre,
#     link_list=larm_link_list,
#     move_target=larm_end_coords)
# ri.angle_vector(robot.angle_vector(), 5)
# ri.wait_interpolation()
