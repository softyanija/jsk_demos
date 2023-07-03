from skrobot.interfaces.ros import PR2ROSRobotInterface
import skrobot
import camera_calib_params
from subscribe_timercam_tf import Calculation_timercam_pos
import argparse
import rospy
import time

rospy.init_node("screw_test")

r = skrobot.models.PR2()
ri = PR2ROSRobotInterface(r)
rarm_link_list = [
    r.r_shoulder_pan_link,
    r.r_shoulder_lift_link,
    r.r_upper_arm_roll_link,
    r.r_elbow_flex_link,
    r.r_forearm_roll_link, 
    r.r_wrist_flex_link,
    r.r_wrist_roll_link]

viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(r)
viewer.show()

rarm_init_coords = skrobot.coordinates.Coordinates([0.5, -0.3, 0.82], [0, 1.57, 0])
rarm_screw_coords = skrobot.coordinates.Coordinates([0.5, -0.3, 0.77], [0, 1.57, 0])

rarm_end_coords = skrobot.coordinates.CascadedCoords(
    parent=r.r_gripper_tool_frame,
    name="rarm_end_coords")
move_target = rarm_end_coords

ri.angle_vector(r.reset_manip_pose(), 5)

ri.move_gripper("rarm", 3.0)

r.inverse_kinematics(
    rarm_init_coords,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()

input("wait for key...")

ri.move_gripper("rarm", 0.0)

r.inverse_kinematics(
    rarm_screw_coords,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()

input("wait for key...")

for i in range(10):
    tmp = r.r_wrist_roll_joint.joint_angle()
    r.r_wrist_roll_joint.joint_angle(tmp + 3.14)
    ri.angle_vector(r.angle_vector(), 2)
    ri.wait_interpolation()

