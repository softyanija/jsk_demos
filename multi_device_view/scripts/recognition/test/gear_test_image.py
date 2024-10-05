import cv2
import sys
import pdb
import math
import numpy as np
import pyrealsense2 as rs


if len(sys.argv) < 2:
    print("Usage: python image_diff.py <image_path>")
    sys.exit(1)

image_path = sys.argv[1]

image = cv2.imread(image_path)

gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray_img = cv2.medianBlur(gray_img, 3)
    
equ_hist = cv2.equalizeHist(gray_img)
equ_img = np.hstack((gray_img, equ_hist))
equ_height, equ_width = equ_img.shape
equ_img_clip = equ_img[:, equ_width//2:equ_width]
_, threshold_image = cv2.threshold(equ_img_clip, 130, 255, cv2.THRESH_BINARY)

#ellipse_contours, hierarcy = cv2.findContours(threshold_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
#ellipse_drawed_image = frame.copy()
#ellipse_list = []
# for i, cnt in enumerate(ellipse_contours):
#     if (len(cnt) >= 5): 
#         ellipse = cv2.fitEllipse(cnt)
#         if (not math.isnan(ellipse[0][0])) and (not math.isnan(ellipse[0][1])) and (not math.isnan(ellipse[1][0])) and (not math.isnan(ellipse[1][1])):
#             ellipse_list.append(ellipse)
#             cx = int(ellipse[0][0])
#             cy = int(ellipse[0][1])
#             h = int(ellipse[1][0])
#             w = int(ellipse[1][1])
#             if (h * w > 300) and (h * w < 2000):
#                 ellipse_drawed_image  = cv2.ellipse(ellipse_drawed_image, ellipse, (255,0,0),2)
#                 ellipse_drawed_image = cv2.putText(ellipse_drawed_image, str(h * w), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,80,255), 1,cv2.LINE_AA)

cv2.imshow("gray", gray_img)
cv2.imshow("equalize", equ_img_clip)
cv2.imshow("thresholded", threshold_image)
# cv2.imshow("ellipse", ellipse_drawed_image)
cv2.imwrite("../debug_image/gear_test/gear_test_gray.png", gray_img)
cv2.imwrite("../debug_image/gear_test/gear_test_equalize.png", equ_img_clip)
cv2.imwrite("../debug_image/gear_test/gear_test_thresholded.png", threshold_image)

cv2.waitKey(0)
cv2.destroyAllWindows()
