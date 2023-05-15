from skrobot.interfaces.ros import PR2ROSRobotInterface
import skrobot
#import camera_calib_motion_prot_params
rarm_init_coords = skrobot.coordinates.Coordinates([0.5, -0.3, 0.75], [0, 1.57, 0])
rarm_calib_pose_0_1 =skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [0, 0, 0.3])
rarm_calib_pose_0_2 =skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [0, 0, -0.3])
rarm_calib_pose_0_3 =skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [0, 0.3, 0])
rarm_calib_pose_0_4 =skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [0, -0.3, 0])
rarm_calib_pose_0_5 =skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [0.3, 0, 0])
rarm_calib_pose_0_6 =skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [-0.3, 0, 0])

r = skrobot.models.PR2()
ri = PR2ROSRobotInterface(r)

viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(r)
viewer.show()

# ri.angle_vector(r.reset_pose())
# ri.angle_vector(r.reset_pose(), 5)

rarm_link_list = [
    r.r_shoulder_pan_link,
    r.r_shoulder_lift_link,
    r.r_upper_arm_roll_link,
    r.r_elbow_flex_link,
    r.r_forearm_roll_link, 
    r.r_wrist_flex_link,
    r.r_wrist_roll_link]

ri.angle_vector(r.reset_manip_pose(), 5)
rarm_end_coords = skrobot.coordinates.CascadedCoords(
    parent=r.r_gripper_tool_frame,
    name="rarm_end_coords")
move_target = rarm_end_coords

r.inverse_kinematics(
    rarm_init_coords,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()

rarm_calib_pose_1 = skrobot.coordinates.base.transform_coords(rarm_init_coords, rarm_calib_pose_0_1)
r.inverse_kinematics(
    rarm_calib_pose_1,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()

rarm_calib_pose_2 = skrobot.coordinates.base.transform_coords(rarm_init_coords, rarm_calib_pose_0_2)
r.inverse_kinematics(
    rarm_calib_pose_2,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()

rarm_calib_pose_3 = skrobot.coordinates.base.transform_coords(rarm_init_coords, rarm_calib_pose_0_3)
r.inverse_kinematics(
    rarm_calib_pose_3,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()

rarm_calib_pose_4 = skrobot.coordinates.base.transform_coords(rarm_init_coords, rarm_calib_pose_0_4)
r.inverse_kinematics(
    rarm_calib_pose_4,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()

rarm_calib_pose_5 = skrobot.coordinates.base.transform_coords(rarm_init_coords, rarm_calib_pose_0_5)
r.inverse_kinematics(
    rarm_calib_pose_5,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()

rarm_calib_pose_6 = skrobot.coordinates.base.transform_coords(rarm_init_coords, rarm_calib_pose_0_6)
r.inverse_kinematics(
    rarm_calib_pose_6,
    link_list=rarm_link_list,
    move_target=rarm_end_coords)
ri.angle_vector(r.angle_vector(), 5)
ri.wait_interpolation()
