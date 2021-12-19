# だいたいEUSLISPでかく
# dataset.saveとmodel.fit, model.predictだけpythonのプログラムにservice callを送って実行する
(defun dataset_save (s a_d)
 (setq req (instance roseus::ActionDeltaData :init))
 (send req :status s)
 (send req :actoin_delta a_d)
 (setq res (ros::service-call "dataset_save" req t))

(defun model_fit)
 (setq req  ...)
 (setq res (ros::service-call "model_fit" req t)))

(defun model_predict (s)
 (setq req (instance roseus::RobotStatus :init))
 (send req :status s)
 (setq action (ros::service-call "model_predict" req t))
 ;; (send *robot* :angle-vector actoin) ;;みたいな感じ
 )

(defun main
    ;; 疑似コードのアルゴリズムをここに実装する
    ...)
