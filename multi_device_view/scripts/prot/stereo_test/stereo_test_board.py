import numpy as np
import cv2
import rospy
from geometry_msgs.msg import *
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
import quaternion
from skrobot.coordinates import Coordinates


cameraMatrix1 = np.array([[471.99697,         0, 299.54147],
                          [0        , 471.4478 , 229.05121],
                          [0        ,         0,   1.     ]])
distCoeffs1 = np.array([[-0.331875, 0.089660, -0.002166, -0.000423, 0.000000]])
cameraMatrix2 = np.array([[489.44513,         0, 334.76048],
                          [0        , 488.25644, 232.22405],
                          [0        ,         0,   1.     ]])
distCoeffs2 = np.array([[-0.385310, 0.143555, 0.000299, 0.001975, 0.000000]])


N = 35 #キャリブレーション用ステレオ画像のペア数
#　　　　「left0.jgp」のように、ペア番号を'left','right'の後につけて同じフォルダに置く(grobが使いこなせれば直したい)

square_size =0.03      # 正方形のサイズ
pattern_size = (6, 5)  # 格子数
pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 ) #チェスボード（X,Y,Z）座標の指定 (Z=0)
pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size
obj_points = []
img_points1 = []
img_points2 = []


for i in range(1, N + 1):
    # 画像の取得
    im_l = cv2.imread("calib_img/checker_board/timercam2_" +str(i)+ ".png", 0)
    im_r = cv2.imread("calib_img/checker_board/timercam1_" +str(i)+ ".png", 0)
    print("loading..." + "timercam2_" +str(i)+ ".png")
    print("loading..." + "timercam1_" +str(i)+ ".png")
    #コーナー検出
    found_l, corner_l = cv2.findChessboardCorners(im_l, pattern_size)
    found_r, corner_r = cv2.findChessboardCorners(im_r, pattern_size)
    # コーナーがあれば
    if found_l and found_r:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        cv2.cornerSubPix(im_l, corner_l, (5,5), (-1,-1), term)
        cv2.cornerSubPix(im_r, corner_r, (5,5), (-1,-1), term)
        cv2.drawChessboardCorners(im_l, pattern_size, corner_l,found_l)
        cv2.drawChessboardCorners(im_r, pattern_size, corner_r,found_r)
        # cv2.imshow('found corners in ' + "left" +str(i)+ ".jpg", im_l)
        # cv2.imshow('found corners in ' + "right" +str(i)+ ".jpg", im_r)
        # cv2.waitKey(10)
    # コーナーがない場合のエラー処理
    if not found_l:
        print('chessboard not found in leftCamera')
        continue
    if not found_r:
        print('chessboard not found in rightCamera')
        continue

    # 選択ボタンを表示
    img_points1.append(corner_l.reshape(-1, 2))
    img_points2.append(corner_r.reshape(-1, 2))
    obj_points.append(pattern_points)
    print('found corners in ' + str(i) + ' is adopted')
cv2.destroyAllWindows()

# システムの外部パラメータを計算
imageSize = (im_l.shape[1],im_l.shape[0])
# cameraMatrix1 = K_l
# cameraMatrix2 = K_r
# distCoeffs1 = d_l
# distCoeffs2 = d_r
retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(
    obj_points, img_points1, img_points2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize)
T_normal = T / np.linalg.norm(T)
# 計算結果を表示
print("retval = ", retval)
print("R = \n", R)
print("T = \n", T)
print("T_normal = \n", T_normal)

# 計算結果を保存
# numpy.savetxt("cameraMatrix1.csv", cameraMatrix1, delimiter =',',fmt="%0.14f") #新しいカメラ行列を保存
# numpy.savetxt("cameraMatrix2.csv", cameraMatrix2, delimiter =',',fmt="%0.14f")
# numpy.savetxt("distCoeffs1.csv", distCoeffs1, delimiter =',',fmt="%0.14f") #新しい歪み係数を保存
# numpy.savetxt("distCoeffs2.csv", distCoeffs2, delimiter =',',fmt="%0.14f")
# numpy.savetxt("R.csv", R, delimiter =',',fmt="%0.14f") #カメラ間回転行列の保存
# numpy.savetxt("T.csv", T, delimiter =',',fmt="%0.14f") #カメラ間並進ベクトルの保存

rospy.init_node("stereo_test", anonymous=True)
rectify_scale = 1





R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify( \
        cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (640, 480), R, T, alpha=rectify_scale)
print(roi1, roi2)
# prepare to remap the webcams
l_maps = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, (640, 480), cv2.CV_16SC2)
r_maps = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, (640, 480), cv2.CV_16SC2)

#i = 1
lframe = cv2.imread("calib_img/stereo_test/timercam2_000001.png")
rframe = cv2.imread("calib_img/stereo_test/timercam1_000001.png")
# use the rectified data to do remap on webcams
lframe_remap = cv2.remap(lframe, l_maps[0], l_maps[1], cv2.INTER_LINEAR)
rframe_remap = cv2.remap(rframe, r_maps[0], r_maps[1], cv2.INTER_LINEAR)
viz = np.concatenate([lframe_remap, rframe_remap], axis=1)
cv2.line(viz, (0,200), (1280, 200), (255, 0, 0))
cv2.line(viz, (0,210), (1280, 210), (255, 0, 0))
cv2.line(viz, (0,220), (1280, 220), (255, 0, 0))
cv2.line(viz, (0,230), (1280, 230), (255, 0, 0))
cv2.line(viz, (0,240), (1280, 240), (255, 0, 0))
cv2.line(viz, (0,250), (1280, 250), (255, 0, 0))
cv2.line(viz, (0,260), (1280, 260), (255, 0, 0))
cv2.imshow('viz', viz)
cv2.imshow('lframe', lframe)
cv2.imshow('rframe', rframe)
cv2.waitKey()
cv2.destroyAllWindows()
