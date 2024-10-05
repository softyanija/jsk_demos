import rospy
import tf
import tf2_ros
import skrobot

from skrobot.coordinates.quaternion import Quaternion
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from dynamic_tf_publisher.srv import SetDynamicTF

class SetModuleTf():

    def __init__(self):
        self.estimated_tf = None
        self.tf_hz = 100
        self.arm_side = None
        self.module = None


    def reset_estimated_tf(self):
        self.estimated_tf = None
        

    def estimate_tf(self, arm_side, module):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        try:
            b2g = tf_buffer.lookup_transform("base_link", arm_side + "_gripper_front", rospy.Time(0), rospy.Duration(3))
            g2m = tf_buffer.lookup_transform(module + "_" + arm_side + "_gripper_front_apriltag", module + "_base", rospy.Time(0), rospy.Duration(3))
            base_to_gripper = skrobot.coordinates.Coordinates([b2g.transform.translation.x,
                                                               b2g.transform.translation.y,
                                                               b2g.transform.translation.z],
                                                              [b2g.transform.rotation.w,
                                                               b2g.transform.rotation.x,
                                                               b2g.transform.rotation.y,
                                                               b2g.transform.rotation.z]
            )
            gripper_to_module = skrobot.coordinates.Coordinates([g2m.transform.translation.x,
                                                                 g2m.transform.translation.y,
                                                                 g2m.transform.translation.z],
                                                                [g2m.transform.rotation.w,
                                                                 g2m.transform.rotation.x,
                                                                 g2m.transform.rotation.y,
                                                                 g2m.transform.rotation.z])

            base_to_module = base_to_gripper.copy_worldcoords().transform(gripper_to_module)

            new_tf = TransformStamped()
            new_tf.header.frame_id = "base_link"
            new_tf.child_frame_id = module + "_base"
            new_tf.transform.translation.x = base_to_module.translation[0]
            new_tf.transform.translation.y = base_to_module.translation[1]
            new_tf.transform.translation.z = base_to_module.translation[2]
            new_tf.transform.rotation.x = base_to_module.quaternion[1]
            new_tf.transform.rotation.y = base_to_module.quaternion[2]
            new_tf.transform.rotation.z = base_to_module.quaternion[3]
            new_tf.transform.rotation.w = base_to_module.quaternion[0]

            self.estimated_tf = new_tf

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get transform")
            self.estimated_tf = None
        

    def set_estimated_tf(self, arm_side, module):

         # self.estimate_tf(arm_side, module)
        
        rospy.wait_for_service("/set_dynamic_tf")
        try:
            client = rospy.ServiceProxy("/set_dynamic_tf", SetDynamicTF)
            res = client(self.tf_hz, self.estimated_tf)
            return
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node("set_module_tf")
    setter = SetModuleTf()
    setter.estimate_tf("l", "module_0")
    setter.set_estimated_tf("l", "module_0")
    setter.estimate_tf("l", "module_1")
    setter.set_estimated_tf("l", "module_1")
