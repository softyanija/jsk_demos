import cv2
import numpy as np

# def triangulation(T_1w, T_2w, kp1, kp2):
#     """Triangulation to get 3D points
#     Args:
#         T_1w (4x4): pose of view 1 w.r.t  i.e. T_1w (from w to 1)
#         T_2w (4x4): pose of view 2 w.r.t world, i.e. T_2w (from w to 2)
#     　　[R(3), t(1)]
#         kp1 (2,1): keypoint in view 1 (not normalized?)
#         kp2 (2,1): keypoints in view 2 (not normalized?
#         shape of T will be change by treatment of tf 
    
#     Returns:
#         X (3,1): 3D coordinates of the keypoints w.r.t world coordinate
#         X1 (3,1): 3D coordinates of the keypoints w.r.t view1 coordinate
#         X2 (3,1): 3D coordinates of the keypoints w.r.t view2 coordinate
#     """
#     X = cv2.triangulatePoints(T_1w[:3], T_2w[:3], kp1, kp2)
#     X = X / X[3]
#     X1 = T_1w[:3] @ X
#     X2 = T_2w[:3] @ X
#     return X[:3], X1, X2

