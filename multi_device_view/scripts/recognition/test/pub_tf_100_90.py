import skrobot
from skrobot.coordinates import Coordinates
import math
import rospy
from geometry_msgs.msg import TransformStamped
from dynamic_tf_publisher.srv import SetDynamicTF


rospy.wait_for_service("/set_dynamic_tf")
try:
    client = rospy.ServiceProxy("/set_dynamic_tf", SetDynamicTF)
    # buf_0 = skrobot.coordinates.Coordinates([0.0707, -0.0707, 0], [0,0,math.pi * 0.75])
    buf_0 = skrobot.coordinates.Coordinates([0.1004, -0.1004, 0], [math.pi * 0.75, 0, 0])
    buf_1 = skrobot.coordinates.Coordinates([0.1004, 0.1004, 0], [-math.pi * 0.75, 0, 0]) 
    module_0 = TransformStamped()
    module_0.header.frame_id = "base_link"
    module_0.child_frame_id = "module_0_base"
    module_0.transform.translation.x = buf_0.translation[0]
    module_0.transform.translation.y = buf_0.translation[1]
    module_0.transform.translation.z = buf_0.translation[2]
    module_0.transform.rotation.x = buf_0.quaternion[1]
    module_0.transform.rotation.y = buf_0.quaternion[2]
    module_0.transform.rotation.z = buf_0.quaternion[3]
    module_0.transform.rotation.w = buf_0.quaternion[0]

    module_1 = TransformStamped()
    module_1.header.frame_id = "base_link"
    module_1.child_frame_id = "module_1_base"
    module_1.transform.translation.x = buf_1.translation[0]
    module_1.transform.translation.y = buf_1.translation[1]
    module_1.transform.translation.z = buf_1.translation[2]
    module_1.transform.rotation.x = buf_1.quaternion[1]
    module_1.transform.rotation.y = buf_1.quaternion[2]
    module_1.transform.rotation.z = buf_1.quaternion[3]
    module_1.transform.rotation.w = buf_1.quaternion[0]
    
    res0 = client(10, module_0)
    res1 = client(10, module_1)


except rospy.ServiceException as e:
    print("Service call failed: %s"%e)
