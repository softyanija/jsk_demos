import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform
import numpy as np
from pyquaternion import Quaternion

#write only class definition in this file, and I will make two python program to launch node


class Calculation_timercam_pos:
    def __init__(self, timercam_name, timercam_camera_frame="base_link"):
        self.timercam_name = timercam_name
        self.timercam_tf = None
        self.timercam_tf_stock = []
        self.timercam_camera_frame = timercam_camera_frame
        self.Rate = 5
        self.subscribe()
        
        
    def subscribe(self):
        rospy.Subscriber("tf", TFMessage, self.callback)
        rate = rospy.Rate(self.Rate)
        
    def callback(self, data):
        #rospy.loginfo("len of data.transforms is %s",len(data.transforms))
        
        for tf in data.transforms:
            if tf.header.frame_id == self.timercam_camera_frame: #timer_cam1_optical_frame
                #print("found tf")
                #print(tf.transform)
                self.timercam_tf = tf.transform
                
            else:
                self.timercam_tf = None
        #print(self.timercam_tf)

    def stock_tf(self):
        if not (self.timercam_tf == None):
            self.timercam_tf_stock.append(self.timercam_tf)
            return True
        else:
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
                pose_x += i.translation.x
                pose_y += i.translation.y
                pose_z += i.translation.z

                q_buf = Quaternion(i.rotation.w, i.rotation.x, i.rotation.y, i.rotation.z)
                qlist.append(q_buf)
                

            pose_x = pose_x / len(self.timercam_tf_stock)
            pose_y = pose_y / len(self.timercam_tf_stock)
            pose_z = pose_z / len(self.timercam_tf_stock)
                
            x = np.array([q.elements for q in qlist])
            m = x.T @ x
            w,v = np.linalg.eig(m)
            #return Quaternion(v[:, np.argmax(w)])
            q_average = Quaternion(v[:, np.argmax(w)])

            tf_average = Transform()
            tf_average.translation.x = pose_x
            tf_average.translation.y = pose_y
            tf_average.translation.z = pose_z
            tf_average.rotation.x = q_average.x
            tf_average.rotation.y = q_average.y
            tf_average.rotation.z = q_average.z
            tf_average.rotation.w = q_average.w

            return tf_average
        
        
        



# def callback(data):
#     rospy.loginfo("len of data.transforms is %s",len(data.transforms))
#     for tf in data.transforms:
#        if tf.header.frame_id == "base_link":
#            print("found base_link")
#            print(tf.transform)
#            return tf.transform

# def get_tf():
#     rospy.init_node("tf_listener", anonymous=True)
#     rospy.Subscriber("tf", TFMessage, callback)
#     rate = rospy.Rate(5)
#     while not rospy.is_shutdown():
#         rate.sleep()

# if __name__ == '__main__':
#     get_tf()
