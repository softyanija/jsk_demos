#!/usr/bin/env python3

import numpy as np
import rospy
import skrobot
import tf
import time
import tf2_ros
import pdb

def get_module_coordinates(module_name, camera_position):
    # camera_position:{rarm, larm, head}
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    module_frame = camera_position + "_" + module_name
    b2m = None

    try:
        b2m = tf_buffer.lookup_transform("base_link", module_frame, rospy.Time(), rospy.Duration(3))
        base_to_module = skrobot.coordinates.Coordinates([b2m.transform.translation.x,
                                                               b2m.transform.translation.y,
                                                               b2m.transform.translation.z],
                                                              [b2m.transform.rotation.w,
                                                               b2m.transform.rotation.x,
                                                               b2m.transform.rotation.y,
                                                               b2m.transform.rotation.z]
            )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to get transform")
        pdb.set_trace()

    return b2m



if __name__ == "__main__":
    rospy.init_node("recog_module", anonymous=True)
    
    module_coordinates = get_module_coordinates("module_0", "rarm")
    
