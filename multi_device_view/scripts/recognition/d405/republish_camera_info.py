import rospy
from sensor_msgs.msg import CameraInfo

 
def callback(msg):
    pub_msg = CameraInfo()
    pub_msg = msg
    pub_msg.header.frame_id = "camera_color_optical_frame_estimated"
    pub_info.publish(pub_msg)
    print(pub_msg)
    

rospy.init_node("republish_camera_info")
    
pub_info = rospy.Publisher("/camera/estimated/camera_info", CameraInfo, queue_size=1)
rospy.Subscriber("/camera/color/camera_info", CameraInfo, callback)

rospy.spin()
