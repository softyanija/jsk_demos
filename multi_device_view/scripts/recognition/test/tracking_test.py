import cv2
import time

# カメラの起動
cap = cv2.VideoCapture(2)  # カメラのデバイス番号を指定（通常は0）

# カメラが正常に起動したかを確認
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

time.sleep(2)
    
# 初期の対象物体の矩形を選択
ret, frame = cap.read()
bbox = cv2.selectROI('Select Object', frame, False)

# MEDIANFLOW Trackerの作成
tracker = cv2.TrackerMIL_create()

# トラッキングの初期化
tracker.init(frame, bbox)

while True:
    # 新しいフレームを取得
    ret, frame = cap.read()

    # トラッキング更新
    ok, new_bbox = tracker.update(frame)

    # トラッキングが成功した場合
    if ok:
        # 矩形を描画
        p1 = (int(new_bbox[0]), int(new_bbox[1]))
        p2 = (int(new_bbox[0] + new_bbox[2]), int(new_bbox[1] + new_bbox[3]))
        cv2.rectangle(frame, p1, p2, (0, 255, 0), 2, 1)
    else:
        # トラッキングが失敗した場合の処理
        print("Tracking failure!")

    # 結果を表示
    cv2.imshow('Tracking', frame)

    # 終了条件（'q'キーを押したら終了）
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# カメラを解放してウィンドウを閉じる
cap.release()
cv2.destroyAllWindows()
