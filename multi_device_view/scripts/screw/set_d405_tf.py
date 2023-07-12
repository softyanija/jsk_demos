import rospy
import tf
import tf2_ros
import skrobot

from skrobot.coordinates.quaternion import Quaternion
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
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
            #trans = tf_buffer.lookup_transform("base_link", "camera_link", rospy.Time(), rospy.Duration(3))
            b2g = tf_buffer.lookup_transform("base_link", "r_gripper_front", rospy.Time(), rospy.Duration(3))
            g2c = tf_buffer.lookup_transform("r_gripper_front_apriltag", "camera_link", rospy.Time(), rospy.Duration(3))
            base_to_gripper = skrobot.coordinates.Coordinates([b2g.transform.translation.x,
                                                               b2g.transform.translation.y,
                                                               b2g.transform.translation.z],
                                                              [b2g.transform.rotation.w,
                                                               b2g.transform.rotation.x,
                                                               b2g.transform.rotation.y,
                                                               b2g.transform.rotation.z]
            )
            gripper_to_camera = skrobot.coordinates.Coordinates([g2c.transform.translation.x,
                                                                 g2c.transform.translation.y,
                                                                 g2c.transform.translation.z],
                                                                [g2c.transform.rotation.w,
                                                                 g2c.transform.rotation.x,
                                                                 g2c.transform.rotation.y,
                                                                 g2c.transform.rotation.z])
            
            # base_to_gripper.translation = [b2g.transform.translation.x,
            #                                b2g.transform.translation.y,
            #                                b2g.transform.translation.z]
            # base_to_gripper.quaternion = [b2g.transform.rotation.w,
            #                               b2g.transform.rotation.x,
            #                               b2g.transform.rotation.y,
            #                               b2g.transform.rotation.z]
            # gripper_to_camera.translation = [g2c.transform.translation.x,
            #                                  g2c.transform.translation.y,
            #                                  g2c.transform.translation.z]
            # gripper_to_camera.quaternion = [g2c.transform.rotation.w,
            #                                 g2c.transform.rotation.x,
            #                                 g2c.transform.rotation.y,
            #                                 g2c.transform.rotation.z]

            base_to_camera = base_to_gripper.copy_worldcoords().transform(gripper_to_camera)
            
            new_tf = TransformStamped()
            new_tf.header.frame_id = "base_link"
            new_tf.child_frame_id = "camera_link"
            new_tf.transform.translation.x = base_to_camera.translation[0]
            new_tf.transform.translation.y = base_to_camera.translation[1]
            new_tf.transform.translation.z = base_to_camera.translation[2]
            new_tf.transform.rotation.x = base_to_camera.quaternion[1]
            new_tf.transform.rotation.y = base_to_camera.quaternion[2]
            new_tf.transform.rotation.z = base_to_camera.quaternion[3]
            new_tf.transform.rotation.w = base_to_camera.quaternion[0]
            
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
