#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy

import tf
import jsk_recognition_msgs.msg

def handle_leg_pose(msg, leg):
    br = tf.TransformBroadcaster()
    rospy.loginfo("hoge")
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "leg",
                     "base_link")

if __name__ == '__main__':
    rospy.init_node('leg_tf_broadcaster')
#    leg = rospy.get_param('~leg')
    rospy.Subscriber('/HSI_color_filter/boxes',
                     jsk_recognition_msgs.msg.BoundingBoxArray,
                     handle_leg_pose,
                     "leg")
    rospy.loginfo("huga")
    # while not rospy.is_shutdown():
    #     handle_leg_pose
    #     rospy.loginfo("publishing")
    #     rospy.sleep(1)
    rospy.spin()
