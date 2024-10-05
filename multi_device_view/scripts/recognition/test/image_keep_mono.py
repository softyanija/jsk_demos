import cv2

# カメラを起動
cap = cv2.VideoCapture(8)  # カメラ番号を指定（0はデフォルトカメラ）

# キー入力待ちのフラグ
key_pressed = False

while True:
    # フレームをキャプチャ
    ret, frame = cap.read()

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # フレームを表示
    cv2.imshow("Video", frame)

    # キー入力を待ちます
    key = cv2.waitKey(1)

    # スペースキーが押された場合
    if key == ord(' '):
        # 画像として保存
        cv2.imwrite("captured_frame.jpg", frame)
        print("Frame captured and saved as captured_frame.jpg")
        key_pressed = True

    # 'q'キーが押された場合、プログラムを終了
    if key == ord('q'):
        break

# キャプチャを解放
cap.release()

# ウィンドウをすべて閉じる
cv2.destroyAllWindows()
