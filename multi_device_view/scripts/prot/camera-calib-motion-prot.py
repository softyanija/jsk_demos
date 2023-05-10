from skrobot.interfaces.ros import PR2ROSRobotInterface
import skrobot

r = skrobot.models.PR2()
ri = PR2ROSRobotInterface(r)

viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640,480))
viewer.add(r)
viewer.show()

ri.angle_vector(r.reset_pose())
ri.angle_vector(r.reset_pose(), 5)

rarm_end_coords = skrobot.coordinates.CascadedCoords(
    parent=r.r_gripper_tool_frame,
    name="rarm_end_coords")
move_target = rarm_end_coords
link_list = [
    r.r_shoulder_pan_link,
    r.r_shoulder_lift_link,
    r.r_upper_arm_roll_link,
    r.r_elbow_flex_link,
    r.r_forearm_roll_link,
    r.r_wrist_flex_link,
    r.r_wrist_roll_link]
target_coords=skrobot.coordinates.Coordinates([0.5, -0.3, 0.7], [0, 0, 0])
r.inverse_kinematics(
    target_coords,
    link_list=link_list,
    move_target=move_target) 
