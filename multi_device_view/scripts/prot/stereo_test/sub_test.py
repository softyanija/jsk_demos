import numpy as np
import cv2
import rospy
from geometry_msgs.msg import *
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage

# def callback(data):
#     print(data)
#     wait_for_message
    
    
# def subscribe():

#     rospy.init_node("sub_test", anonymous=True)
#     rospy.Subscriber("tf", TFMessage, callback)
#     rospy.spin()


rospy.init_node("sub_test", anonymous=True)
f = 0

tf_buffer_1 = tf2_ros.Buffer()
tf_buffer_2 = tf2_ros.Buffer()
tf_listerner_1 = tf2_ros.TransformListener(tf_buffer_1)
tf_listerner_2 = tf2_ros.TransformListener(tf_buffer_2)

tf_buffer = tf2_ros.Buffer()
tf_listerner = tf2_ros.TransformListener(tf_buffer)

while(1):
    try:
        trans = tf_buffer_1.lookup_transform("timercam_2_optical_frame_estimated", "timercam_1_optical_frame_estimated", rospy.Time(), rospy.Duration(0.5))
        break
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to get trans")
        
# while(1):
#     try:
#         trans1 = tf_buffer_1.lookup_transform("base_link", "timercam_1_optical_frame_estimated", rospy.Time(), rospy.Duration(0.5))
#         break
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#         rospy.logwarn("Failed to get trans1")

# while(1):
#     try:
#         trans2 = tf_buffer_2.lookup_transform("base_link", "timercam_2_optical_frame_estimated", rospy.Time(), rospy.Duration(0.5))
#         break
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#         rospy.logwarn("Failed to get trans1")
        
 

#data = rospy.wait_for_message("/tf", TFMessage, timeout = 2)

    
