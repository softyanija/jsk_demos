import cv2
import sys
import pdb
import math
import numpy as np

image = cv2.imread("../debug_image/get_rect_test.png")
image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15))
dilation = cv2.morphologyEx(image_gray, cv2.MORPH_CLOSE, kernel) 


cv2.imshow("origin", image)
cv2.imshow("dilation", dilation)

# pdb.set_trace()

cv2.waitKey(0)
cv2.destroyAllWindows()
