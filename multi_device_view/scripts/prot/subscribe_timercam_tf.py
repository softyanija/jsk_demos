import rospy
from tf2_msgs.msg import TFMessage

#write only class definition in this file, and I will make two python program to launch node

class Calculation_timercam_pos:
    def __init__(self, timercam_name):
        self.timercam_name = timercam_name
        self.timercam_tf = None
        self.Rate = 5
        self.subscribe()
        
    def subscribe(self):
        rospy.Subscriber("tf", TFMessage, self.callback)
        rate = rospy.Rate(self.Rate)
        
    def callback(self, data):
        rospy.loginfo("len of data.transforms is %s",len(data.transforms))
        for tf in data.transforms:
            if tf.header.frame_id == "base_link": #timer_cam1_optical_frame
                #print("found tf")
                #print(tf.transform)
                self.timercam_tf = tf.transform
                #return tf.transform
                #tfの入れ物作ってほしいtfが見つかったらそれを返しなかったらNoneを返す
        #print(self.timercam_tf)


        
        
        



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
                  
