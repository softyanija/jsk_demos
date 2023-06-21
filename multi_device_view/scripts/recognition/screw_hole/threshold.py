import cv2
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--image")
parser.add_argument("--threshold", "-t", default=100, type = int)
args = parser.parse_args()

img = cv2.imread(args.image, 0)

threshold = args.threshold

ret, img_threshold = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)

cv2.imshow("img_thresh", img_threshold)
cv2.waitKey()
cv2.destroyAllwindows()
