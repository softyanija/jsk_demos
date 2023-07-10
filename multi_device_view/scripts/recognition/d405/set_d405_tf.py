import rospy
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import *
from dynamic_tf_publisher.srv import SetDynamicTF

class Set_d405_tf:
    def __init__(self):
        self.estimated_tf = None
        self.Rate = 5
        self.tf_hz = 10

    def estimate_tf(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            trans = tf_buffer.lookup_transform("base_link", "camera_link", rospy.Time(), rospy.Duration(3))
            new_tf = TransformStamped()
            new_tf.header.frame_id = "base_link"
            new_tf.child_frame_id = "camera_link"
            new_tf.transform = trans.transform
            self.estimated_tf = new_tf

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get transform")

    def set_estimated_tf(self):
        rospy.wait_for_service("/set_dynamic_tf")
        try:
            client = rospy.ServiceProxy("/set_dynamic_tf", SetDynamicTF)
            res = client(self.tf_hz, self.estimated_tf)
            return
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)



if __name__ == "__main__":
    rospy.init_node("set_d405_tf")
    setter = Set_d405_tf()
    setter.estimate_tf()
    setter.set_estimated_tf()
