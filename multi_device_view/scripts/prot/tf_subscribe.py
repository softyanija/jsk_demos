import rospy
from tf2_msgs.msg import TFMessage

def callback(data):
    rospy.loginfo("Get tf")
    rospy.loginfo(len(data.transforms))
    for tf in data.transforms:
       if tf.header.frame_id == "base_link":
           print("found base_link")
           print(tf.transform)

def get_tf():
    rospy.init_node("tf_listener", anonymous=True)
    rospy.Subscriber("tf", TFMessage, callback)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    get_tf()
                  
