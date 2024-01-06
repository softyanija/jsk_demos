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


rospy.init_node("calib_test")
setter = SetModuleTf()
setter.set_estimated_tf("l", "module_0")
setter.set_estimated_tf("l", "module_1")

rate = rospy.Rate(5)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
upper_arm_hole_point_pub = rospy.Publisher("upper_arm_hole/guide_point", PointStamped, queue_size=5)
servo_gear_point_pub = rospy.Publisher("servo_gear/target_point", PointStamped, queue_size=5)
upper_arm_hole_stereo_view = StereoView("module_0/upper_arm_hole/guide_point", "module_1/upper_arm_hole/guide_point")
servo_gear_stereo_view = StereoView("module_0/servo_gear/target_point", "module_1/servo_gear/target_point")

module_0_tf = tf_buffer.lookup_transform("module_0_color_optical_frame", "module_0_color_optical_frame", rospy.Time(0), rospy.Duration(3))
module_0_to_1_tf = tf_buffer.lookup_transform("module_0_color_optical_frame", "module_1_color_optical_frame", rospy.Time(0), rospy.Duration(3))
time.sleep(2)

T_1w = tf2mat(module_0_tf)                                     
T_2w = tf2mat(module_0_to_1_tf)                                     
P_1w = np.dot(servo_gear_stereo_view.K_1, T_1w)                           
P_2w = np.dot(servo_gear_stereo_view.K_2, T_2w)

while not rospy.is_shutdown():
    try:
        rate.sleep()
    except rospy.ROSTimeMovedBackwardsException as e:
        rospy.logwarn("cought {}".format(e))
        pass
        
    upper_arm_hole_X, upper_arm_hole_X1, upper_arm_hole_X2 = stereo_view.triangulation(P_1w, P_2w, upper_arm_hole_stereo_view.kp1, upper_arm_hole_stereo_view.kp2)
    servo_gear_X, servo_gear_X1, servo_gear_X2 = stereo_view.triangulation(P_1w, P_2w, servo_gear_stereo_view.kp1, servo_gear_stereo_view.kp2)

    upper_arm_hole_point_msg = PointStamped()
    upper_arm_hole_point_msg.header.stamp =rospy.Time.now() 
    upper_arm_hole_point_msg.header.frame_id = "module_0_color_optical_frame"

    upper_arm_hole_point_msg.point.x = X[0]
    upper_arm_hole_point_msg.point.y = X[1]
    upper_arm_hole_point_msg.point.z = X[2]

    servo_gear_point_msg = PointStamped()
    servo_gear_point_msg.header.stamp =rospy.Time.now() 
    servo_gear_point_msg.header.frame_id = "module_0_color_optical_frame"

    servo_gear_point_msg.point.x = X[0]
    servo_gear_point_msg.point.y = X[1]
    servo_gear_point_msg.point.z = X[2]

    servo_gear_point_pub.publish(point_msg)
    upper_arm_hole_point_pub.publish(point_msg)

