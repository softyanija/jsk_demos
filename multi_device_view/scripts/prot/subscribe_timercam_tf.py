import rospy
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform
import numpy as np
from pyquaternion import Quaternion

# write only class definition in this file, and I will make two python program to launch node


class Calculation_timercam_pos:
    def __init__(self, timercam_name, timercam_camera_frame="base_link"):
        self.timercam_name = timercam_name
        self.timercam_tf = None
        self.timercam_tf_stock = []
        self.estimated_tf = None
        self.timercam_camera_frame = timercam_camera_frame
        self.Rate = 5
        self.subscribe()
        # self.pub_estimated_tf = rospy.Publisher("/{}_estimated".format(self.timercam_camera_frame), Transform, queue_size=10)

    def subscribe(self):
        rospy.Subscriber("tf", TFMessage, self.callback)
        rate = rospy.Rate(self.Rate)
        rate.sleep()

    def callback(self, data):
        # rospy.loginfo("len of data.transforms is %s",len(data.transforms))
        # may be this func have not to do anything

        for tf in data.transforms:
            if tf.header.frame_id == self.timercam_camera_frame:  # timer_cam1_optical_frame
                # print("found tf")
                # print(tf.transform)
                self.timercam_tf = tf.transform

            else:
                self.timercam_tf = None
        # print(self.timercam_tf)

    def stock_tf(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        try:
            trans = tf_buffer.lookup_transform("base_link", self.timercam_camera_frame, rospy.Time(), rospy.Duration(0.5))
            self.timercam_tf_stock.append(trans)
            return True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get transform")
            return False

    def calculation_tf_average(self):
        if self.timercam_tf_stock == []:
            print("There is no tf in list")
        else:
            pose_x = 0
            pose_y = 0
            pose_z = 0
            qlist = []

            for i in self.timercam_tf_stock:
                pose_x += i.transform.translation.x
                pose_y += i.transform.translation.y
                pose_z += i.transform.translation.z

                q_buf = Quaternion(i.transform.rotation.w, i.transform.rotation.x, i.transform.rotation.y, i.transform.rotation.z)
                qlist.append(q_buf)

            pose_x = pose_x / len(self.timercam_tf_stock)
            pose_y = pose_y / len(self.timercam_tf_stock)
            pose_z = pose_z / len(self.timercam_tf_stock)

            x = np.array([q.elements for q in qlist])
            m = x.T @ x
            w, v = np.linalg.eig(m)
            q_average = Quaternion(v[:, np.argmax(w)])

            tf_average = Transform()
            tf_average.translation.x = pose_x
            tf_average.translation.y = pose_y
            tf_average.translation.z = pose_z
            tf_average.rotation.x = q_average.x
            tf_average.rotation.y = q_average.y
            tf_average.rotation.z = q_average.z
            tf_average.rotation.w = q_average.w

            self.estimated_tf = tf_average
            return tf_average

    # def publish_estimated_tf(self):
    #     self.pub_estimated_tf(self.estimated_tf)
