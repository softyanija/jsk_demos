#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy

import tf
import jsk_recognition_msgs.msg

def handle_leg_pose(msg, leg):
    br = tf.TransformBroadcaster()
    if not ((msg.boxes == []) or (msg.boxes[0].pose.orientation.w == 0.0)):
        br.sendTransform((msg.boxes[0].pose.position.x, msg.boxes[0].pose.position.y, msg.boxes[0].pose.position.z),
                     (msg.boxes[0].pose.orientation.x, msg.boxes[0].pose.orientation.y, msg.boxes[0].pose.orientation.z, msg.boxes[0].pose.orientation.w),
                     rospy.Time.now(),
                     "leg",
                     "head_mount_kinect_rgb_optical_frame")

if __name__ == '__main__':
    rospy.init_node('leg_tf_broadcaster')

    try:
        rospy.Subscriber('/HSI_color_filter/boxes',
                     jsk_recognition_msgs.msg.BoundingBoxArray,
                     handle_leg_pose,
                     "leg")
        rospy.spin()
    except rospy.ROSInterruptException: pass
