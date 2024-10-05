import numpy as np
import rospy
import time
import tf2_ros
import skrobot
import rospkg
import os
import sys

import skrobot.coordinates
from jsk_recognition_msgs.msg import RotatedRectStamped
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from scipy import linalg

rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path('multi_device_view') , "scripts/recognition"))
from set_module_tf import SetModuleTf
from stereo_view import StereoView


rospy.init_node("stereo_test")
setter = SetModuleTf()
setter.estimate_tf("l", "module_0")
setter.set_estimated_tf("l", "module_0")
setter.estimate_tf("l", "module_1")
setter.set_estimated_tf("l", "module_1")


tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

stereo_view = StereoView("servo_gear", "upper_arm_hole")

stereo_view.run()

