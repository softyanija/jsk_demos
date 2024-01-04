import cv2
import pdb
import numpy as np

"""
diff = cv2.absdiff(image1_gray, image2_gray)
diff = cv2.blur(diff, (5,5))
_, thresholded = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)

contours, hierarchy = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#pdb.set_trace()
roi_image = cv2.cvtColor(thresholded.copy(), cv2.COLOR_GRAY2BGR)
"""

image = np.zeros((480, 640, 3), dtype=np.uint8)
center = (320, 240)
axis = (150, 100)
angle = 0
text = "({},{}),{}".format(axis[0], axis[1], angle)
text_center = (center[0]-axis[0]//2, center[1]-axis[1]//3)
cv2.ellipse(image, center, axis, angle, 0, 360, (255, 255, 255), -1)

cv2.putText(image, text, text_center, cv2.FONT_HERSHEY_PLAIN, 1.5, (0,0,0), 2, cv2.LINE_AA)

cv2.imshow("Ellipse", image)
cv2.imwrite("../debug_image/ellipse_test/ellipse_{}_{}_{}.png".format(axis[0], axis[1], angle), image)
cv2.waitKey(0)
cv2.destroyAllWindows()
