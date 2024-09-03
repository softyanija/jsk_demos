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


def tf2mat(msg):
    if msg is None:
        rospy.logwarn("can't recieve tf")

    tf = msg.transform
    tf_coordinate = skrobot.coordinates.Coordinates([tf.translation.x, tf.translation.y, tf.translation.z], [tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z])
    mat = tf_coordinate.T()[:3]
    
    return mat


rospy.init_node("stereo_recognition")

# setter = SetModuleTf()
# setter.set_estimated_tf("l", "module_0")
# setter.set_estimated_tf("l", "module_1")

rate = rospy.Rate(5)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
point_pub = rospy.Publisher("caliblation_debug_point", PointStamped, queue_size=5)
stereo_view = StereoView("module_0/servo_gear/target_point", "module_1/servo_gear/target_point")

module_0_tf = tf_buffer.lookup_transform("module_0_color_optical_frame", "module_0_color_optical_frame", rospy.Time(0), rospy.Duration(3))
module_0_to_1_tf = tf_buffer.lookup_transform("module_0_color_optical_frame", "module_1_color_optical_frame", rospy.Time(0), rospy.Duration(3))
time.sleep(2)                                                  
T_1w = tf2mat(module_0_tf)                                     
T_2w = tf2mat(module_0_to_1_tf)                                     
P_1w = np.dot(stereo_view.K_1, T_1w)                           
P_2w = np.dot(stereo_view.K_2, T_2w)

while not rospy.is_shutdown():
    try:
        rate.sleep()
    except rospy.ROSTimeMovedBackwardsException as e:
        rospy.logwarn("cought {}".format(e))
        pass
        
    X, X1, X2 = stereo_view.triangulation(P_1w, P_2w, stereo_view.kp1, stereo_view.kp2)

    point_msg = PointStamped()
    point_msg.header.stamp =rospy.Time.now() 
    point_msg.header.frame_id = "module_0_color_optical_frame"

    point_msg.point.x = X[0]
    point_msg.point.y = X[1]
    point_msg.point.z = X[2]

    # point_msg.point.x = X1[0]
    # point_msg.point.y = X1[1]
    # point_msg.point.z = X1[2]
        
    point_pub.publish(point_msg)

