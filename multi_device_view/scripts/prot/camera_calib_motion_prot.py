from skrobot.interfaces.ros import PR2ROSRobotInterface
import skrobot
import camera_calib_params
from subscribe_timercam_tf import Calculation_timercam_pos
import argparse
import rospy
import time

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true")
args = parser.parse_args()

if args.test:
    frame_name = "base_link"
else:
    frame_name = "timercam_1_optical_frame"
    
rospy.init_node("sucscribe_timercam_1_tf")
calculation_timercam_pos_1 = Calculation_timercam_pos(timercam_name = "timer_cam1", timercam_camera_frame = frame_name)

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

# ri.angle_vector(r.reset_pose())
# ri.angle_vector(r.reset_pose(), 5)

ri.angle_vector(r.reset_manip_pose(), 5)
rarm_end_coords = skrobot.coordinates.CascadedCoords(
    parent=r.r_gripper_tool_frame,
    name="rarm_end_coords")
move_target = rarm_end_coords

r.inverse_kinematics(
    camera_calib_params.rarm_init_coords,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()

rarm_calib_pose_1 = skrobot.coordinates.base.transform_coords(camera_calib_params.rarm_init_coords, camera_calib_params.rarm_calib_pose_0_1)
r.inverse_kinematics(
    rarm_calib_pose_1,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()
while (not calculation_timercam_pos_1.stock_tf()):
    print("now getting timercam tf at pose 1")
    time.sleep(1)

rarm_calib_pose_2 = skrobot.coordinates.base.transform_coords(camera_calib_params.rarm_init_coords, camera_calib_params.rarm_calib_pose_0_2)
r.inverse_kinematics(
    rarm_calib_pose_2,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()
while (not calculation_timercam_pos_1.stock_tf()):
    print("now getting timercam tf at pose 2")
    time.sleep(1)

rarm_calib_pose_3 = skrobot.coordinates.base.transform_coords(camera_calib_params.rarm_init_coords, camera_calib_params.rarm_calib_pose_0_3)
r.inverse_kinematics(
    rarm_calib_pose_3,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()
while (not calculation_timercam_pos_1.stock_tf()):
    print("now getting timercam tf at pose 3")
    time.sleep(1)

rarm_calib_pose_4 = skrobot.coordinates.base.transform_coords(camera_calib_params.rarm_init_coords, camera_calib_params.rarm_calib_pose_0_4)
r.inverse_kinematics(
    rarm_calib_pose_4,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()
while (not calculation_timercam_pos_1.stock_tf()):
    print("now getting timercam tf at pose 4")
    time.sleep(1)

rarm_calib_pose_5 = skrobot.coordinates.base.transform_coords(camera_calib_params.rarm_init_coords, camera_calib_params.rarm_calib_pose_0_5)
r.inverse_kinematics(
    rarm_calib_pose_5,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()
while (not calculation_timercam_pos_1.stock_tf()):
    print("now getting timercam tf at pose 5")
    time.sleep(1)

rarm_calib_pose_6 = skrobot.coordinates.base.transform_coords(camera_calib_params.rarm_init_coords, camera_calib_params.rarm_calib_pose_0_6)
r.inverse_kinematics(
    rarm_calib_pose_6,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()
while (not calculation_timercam_pos_1.stock_tf()):
    print("now getting timercam tf at pose 6")
    time.sleep(1)

print(calculation_timercam_pos_1.calculation_tf_average())
