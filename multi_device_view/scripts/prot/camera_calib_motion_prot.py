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
    frame_name_1 = "base_link"
    frame_name_2 = "base_link"
else:
    frame_name_1 = "timercam_1_optical_frame"
    frame_name_2 = "timercam_2_optical_frame"
    
rospy.init_node("sucscribe_timercams_tf")
calculation_timercam_pos_1 = Calculation_timercam_pos(timercam_name = "timercam_1", timercam_camera_frame = frame_name_1)
calculation_timercam_pos_2 = Calculation_timercam_pos(timercam_name = "timercam_2", timercam_camera_frame = frame_name_2)

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

pose_number = 6
rarm_calib_poses = []
for i in range(pose_number):
    rarm_calib_pose_buf = skrobot.coordinates.base.transform_coords(camera_calib_params.rarm_init_coords, camera_calib_params.rarm_calib_poses_relative[i])
    r.inverse_kinematics(
        rarm_calib_pose_buf,
        link_list=rarm_link_list,
        move_target=rarm_end_coords)
    ri.angle_vector(r.angle_vector(), 5)
    ri.wait_interpolation()

    n_1 = 0
    while ((not calculation_timercam_pos_1.stock_tf()) and n_1 < 5):
        print("now getting timercam_1 tf at pose {}".format(i))
        n_1+=1
        time.sleep(1)

    n_2 = 0
    while ((not calculation_timercam_pos_2.stock_tf()) and n_2 < 5):
        print("now getting timercam_2 tf at pose {}".format(i))
        n_2+=1
        time.sleep(1)

print(calculation_timercam_pos_1.calculation_tf_average())
print(calculation_timercam_pos_2.calculation_tf_average())

calculation_timercam_pos_1.set_estimated_tf()
calculation_timercam_pos_2.set_estimated_tf()
