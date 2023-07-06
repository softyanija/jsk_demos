import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

if __name__ == "__main__":
    rospy.init_node("apriltag_tf_converter")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Get tf")
            trans = tf_buffer.lookup_transform("r_gripper_front_apriltag", "camera_link", rospy.Time(), rospy.Duration(3))
            new_tf = TransformStamped()
            new_tf.header.stamp = rospy.Time.now()
            new_tf.header.frame_id = "r_gripper_front"
            new_tf.child_frame_id = "camera_link"
            new_tf.transform = trans.transform
            print(trans.transform)
            tf_broadcaster.sendTransform(new_tf)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            
            rospy.logwarn("Failed to get transform from camera")
            rate.sleep()
            
            
