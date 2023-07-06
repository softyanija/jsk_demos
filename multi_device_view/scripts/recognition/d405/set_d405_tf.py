import rospy
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import *
from dynamic_tf_publisher.srv import SetDynamicTF

rospy.init_node("set_d405_tf")

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
tf_broadcaster = tf2_ros.TransformBroadcaster()

rate = 10.0
trans = TransformStamped().transform

while trans == TransformStamped().transform:
    trans = tf_buffer.lookup_transform("base_link", "camera_color_optical_frame", rospy.Time(), rospy.Duration(3))
        
new_tf = TransformStamped()
#new_tf.header.stamp = rospy.Time.now()
new_tf.header.frame_id = "base_link"
new_tf.child_frame_id = "camera_color_optical_frame_estimated"
new_tf.transform = trans.transform

rospy.wait_for_service("/set_dynamic_tf")
try:
    client = rospy.ServiceProxy("/set_dynamic_tf", SetDynamicTF)
    print(new_tf.transform)
    res = client(rate, new_tf)
    
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)
