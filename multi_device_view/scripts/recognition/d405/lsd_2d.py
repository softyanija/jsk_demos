import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("lsd")

def callback(sub_image):

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(sub_image, "bgr8")
    image_gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)

    lines, width, prec, nfa = lsd.detect(image_gray)

    drawnLines = np.zeros((image_gray.shape[0],image_gray.shape[1],3), np.uint8)
    lsd.drawSegments(drawnLines, lines)
    drawnLines = bridge.cv2_to_imgmsg(drawnLines, "bgr8")
    pub_lines.publish(drawnLines)
    #cv2.imshow("Line Segment Detector(LINE)", drawnLines)

    lsd.drawSegments(image, lines)
    colorimg = bridge.cv2_to_imgmsg(image, "bgr8")
    pub_image.publish(colorimg)
    #cv2.imshow("Line Segment Detector", colorimg)


pub_lines = rospy.Publisher("/detection/lsd/lines", Image, queue_size=1)
pub_image = rospy.Publisher("detectoin/lsd/image", Image, queue_size=1)
sub_image = rospy.Subscriber("/camera/color/image_rect_color", Image, callback)

rospy.spin()
