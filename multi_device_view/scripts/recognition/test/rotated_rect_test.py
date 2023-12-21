import cv2
import numpy as np
import pdb

# 画像を作成
image = np.zeros((400, 400, 3), dtype=np.uint8)

# 四角形の頂点を定義
rect_points = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.int32)

# 四角形を回転させるための変換行列を作成
angle = 45.0  # 45度回転
center = (150, 150)  # 回転の中心座標
rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)

# 四角形の各頂点に変換行列を適用して回転
rect_points_rotated = cv2.transform(np.array([rect_points]), rotation_matrix)[0]

# 描画
cv2.polylines(image, [rect_points_rotated], isClosed=True, color=(0, 255, 0), thickness=2)
pdb.set_trace()

# 結果を表示
cv2.imshow('Rotated Rectangle', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
