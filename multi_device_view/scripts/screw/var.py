import numpy as np
import rospy
import skrobot
import tf
import tf2_ros
import time
import math

from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.coordinates.quaternion import Quaternion
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords
from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped, Point, TransformStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from dynamic_tf_publisher.srv import SetDynamicTF
from numpy import pi
from screw_hanged import ScrewHanged

rospy.init_node("var")
screw_hanged = ScrewHanged()
screw_hanged_subscriber = rospy.Subscriber("/screw/euclidean_clustering_decomposer/boxes", BoundingBoxArray, screw_hanged.cb)
rospy.sleep(1)
screw_hanged.calc_tip()
screw_hanged.pub_tip_frame_tf()

tip_list = np.array()
for i in range(100):
    screw_hanged.calc_tip()
    buf = screw_hanged.tip_frame.translation.reshape(3,1)
    if i == 0:
        tip_list = buf
    else:
        tip_list = np.append(tip_list, buf, axis=1)

print(std = np.std(tip_list))

