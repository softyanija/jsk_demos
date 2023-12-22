# -*- coding: utf-8 -*

import numpy as np
import rospy
import skrobot
import tf
import tf2_ros
import time
import math

from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped
from jsk_recognition_msgs.msg import BoundingBoxArray

def get_tag_coordinates(tag_name, camera_position):
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
        pdb.set_trace()

    return b2t


rospy.init_node("inducdtion_screw")
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

init_angle_vector = array([ 0,  0,  0,  0,  0,
                            0,  0,  0,  0,  0,
                            0.        ,  0.        ,  0.3       ,  0.        ,  0.        ,
                            0.87266463,  0.        , -0.9      ,  0.2       , -1.65      ,
                            0.34906584, -1.5       , -0.17453292, -0.17453292,  0.        ,
                            0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
                            0.        ,  0.9      ,  0.2       ,  1.65      , -0.35      ,
                            -1.5       , -0.17453292, -0.17453292,  0.        ,  0.        ,
                            0.        ,  0.        ,  0.        ,  0.        ,  0.        ],
                          dtype=float32)


robot.angle_vector(ri.angle_vector())
robot.angle_vector(init_angle_vector)
ri.angle_vector(robot.angle_vector(), 4)
ri.wait_interpolation()

#move position to see kxr_arm
tag_coordinates = get_tag_coordinates("kxr_arm", "rarm")



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
