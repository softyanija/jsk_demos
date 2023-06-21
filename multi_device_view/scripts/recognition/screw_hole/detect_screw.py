import cv2
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--image")
parser.add_argument("--threshold", "-t", default=100, type = int)
args = parser.parse_args()

img = cv2.imread(args.image, cv2.IMREAD_COLOR)
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
threshold = args.threshold

ret, img_threshold = cv2.threshold(img_gray, threshold, 255, cv2.THRESH_BINARY)
img_threshold_show = cv2.cvtColor(img_threshold, cv2.COLOR_GRAY2BGR)

im_h = cv2.hconcat([img, img_threshold_show])

lsd = cv2.createLineSegmentDetector(quant=6, ang_th = 35)
lines_origin = lsd.detect(img_gray)[0]
lines_gray = lsd.detect(img_threshold)[0]
drawn_line_origin = lsd.drawSegments(img, lines_origin)
drawn_line_gray = lsd.drawSegments(img_threshold, lines_gray)
drawn_line = cv2.hconcat([drawn_line_origin, drawn_line_gray])

# cv2.imshow("im_h", im_h)
# cv2.imshow("drawn_line", drawn_line)
# cv2.imshow("line_origin", drawn_line_origin)

img_fin = cv2.vconcat([im_h, drawn_line])
cv2.imshow("img_fin", img_fin)
cv2.imwrite("./image_log/detect_screw_mono_" + str(threshold) + ".png", img_fin)
cv2.waitKey()
cv2.destroyAllWindows()
