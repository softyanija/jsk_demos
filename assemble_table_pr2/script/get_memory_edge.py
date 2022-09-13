import cv2
import numpy as np

im = cv2.imread('mask_image.jpg',0)
ret,thresh = cv2.threshold(im,127,255,0)
imgEdge,contours,hierarchy = cv2.findContours(thresh, 1, 2)
cnt = contours[4]
rect = cv2.minAreaRect(cnt)

print(rect)
box = cv2.boxPoints(rect)
box = np.int0(box)
print(box.shape)
output = cv2.drawContours(im,[box],0,(0,255,255),5)

cv2.imwrite('./memory_edge_o.jpg',output)
