import numpy as np
import skrobot
from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from geometry_msgs.msg import TransformStamped
import math

# whole
init_angle_vector = np.array([ 0,  0,  0,  0,  0,
                               0,  0,  0,  0,  0,
                               0,  0,  0.3,  0,  0,
                               0.87266463,  0,  -0.9,  0.2,  -1.65,
                               0.34906584,  -1.5,  -0.17453292, -0.17453292,  0,
                               0,  0,  0,  0,  0,
                               0,  0.9,  0.2,  1.65,  -0.35,
                               -1.5, -0.17453292, -0.17453292,  0,  0,
                               0,  0,  0,  0,  0],
                             dtype="float32")

search_arm_vector = np.array([ 0,  0,  0,  0,  0,
                               0,  0,  0,  0,  0,
                               0,  0,  0.3,  0,  0,
                               0.87266463,  0,  -0.9,  0.2,  -1.65,
                               0.34906584,  -1.5,  -0.17453292, -0.17453292,  0,
                               0,  0,  0,  0,  0,
                               0,  0.9,  0.2,  1.65,  -0.35,
                               -1.5, -0.17453292, -0.17453292,  0,  0,
                               0,  0,  0,  0,  0],
                             dtype="float32")

# rarm
search_arm_rarm_vector = np.array([-0.3813113, 0.06809891, -2.1100695, -1.7763098, -1.6647813 , -0.27111033, -0.15205745], dtype="float32")

# larm
search_stored_module_larm_vector = np.array([ 0.85653204, 0.02427874, 1.7829458, -1.577974, 1.6560464, -0.32999033, 3.302054], dtype="float32")

grasp_module_neutral_larm_vector = np.array([ 0.9475635, 0.01040518, 1.4659243, -1.3928123, 1.5354931, -1.5850052, 2.588725], dtype="float32")


#tags
# kxr_tag_to_grasp_pos = skrobot.coordinates.Coordinates([-0.000, 0.001, 0.059], [-3.1, 0.9, -3.1])
kxr_tag_to_grasp_pos = skrobot.coordinates.Coordinates([0.010, 0.001, 0.059], [-3.1, 0.9, -3.1])
module_tag_to_pre_grasp_pos = skrobot.coordinates.Coordinates([-0.007, 0.000, 0.110], [-2.9, 1.5, 1.8])

servo_gear_to_calib_inter_pose = skrobot.coordinates.Coordinates([-0.204, 0.018, 0.156], [0.8, 1.0, -0.1])
servo_gear_to_calib_pose = skrobot.coordinates.Coordinates([-0.036, -0.039, 0.102], [1.8, 1.3, 0.2])



# arange potision
servo_gear_to_module_0 = skrobot.coordinates.Coordinates([0.1004, 0.1004, 0], [-math.pi * 0.75, 0, 0])
servo_gear_to_module_1 = skrobot.coordinates.Coordinates([-0.1004, 0.1004, 0], [-math.pi * 0.25, 0, 0]) 


# for tf
kxr_arm_to_servo_gear = skrobot.coordinates.Coordinates([-0.075, 0.004, 0.005], [0, 0, 0])
