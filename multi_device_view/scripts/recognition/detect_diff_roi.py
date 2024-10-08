import cv2
import sys
import pdb
import numpy as np

if len(sys.argv) < 3:
    print("Usage: python image_diff.py <image1_path> <image2_path>")
    sys.exit(1)

image1_path = sys.argv[1]
image2_path = sys.argv[2]

image1 = cv2.imread(image1_path)
image2 = cv2.imread(image2_path)

image1_gray = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
image2_gray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

diff = cv2.absdiff(image1_gray, image2_gray)
diff = cv2.blur(diff, (5,5))
_, thresholded = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)

contours, hierarchy = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#pdb.set_trace()
roi_image = cv2.cvtColor(thresholded.copy(), cv2.COLOR_GRAY2BGR)

size_buff = 0
index_buff = None
for i, cnt in enumerate(contours):
    _, _, width, height = cv2.boundingRect(cnt)
    if size_buff < width * height:
        size_buff = width * height
        index_buff = i

x, y, width, height = cv2.boundingRect(contours[index_buff])
cv2.rectangle(roi_image, (x, y), (x + width, y + height), color=(0, 255, 0), thickness=4)

edges = cv2.Canny(diff, 50, 150)
circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                           param1=50, param2=30, minRadius=5, maxRadius=100)

diff_copy = diff.copy()
if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:
        cv2.circle(diff_copy, (i[0], i[1]), i[2], (0, 255, 0), 2)
        cv2.circle(diff_copy, (i[0], i[1]), 2, (0, 0, 255), 3)

cv2.imshow("Difference raw", diff)
cv2.imshow("Difference roi", roi_image)
cv2.imshow("Difference_edges", edges)
cv2.imshow("Difference_circle", diff_copy)
cv2.imwrite("../debug_image/detect_diff_raw.png", diff)
cv2.imwrite("../debug_image/detect_diff_roi.png", roi_image)
cv2.imwrite("../debug_image/detect_diff_edges.png", edges)
cv2.imwrite("../debug_image/detect_diff_circles.png", diff_copy)
cv2.waitKey(0)
cv2.destroyAllWindows()
