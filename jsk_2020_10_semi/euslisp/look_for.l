#!/usr/bin/env roseus

(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")


(unless (boundp '*scene*)
  (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
  (fetch-init))

;;棚を障害物として設定
(defun add-shelf-collision-object ()
  (let ((shelf (make-cube 10 1000 1100)))
    (send shelf :translate #f(870 0 550))
    (send *co* :add-object shelf :frame-id "base_link" :object-id "shelf")))

;;それを削除
(defun delete-shelf-collision-object ()
  (send *co* :delete-object-by-id "shelf"))

(add-shelf-collision-object)

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
(send *ri* :wait-interpolation)

;どの引き出しを開けるかの音声認識
(ros::load-ros-manifest "speech_recognition_msgs")
(setq message (one-shot-subscribe "/speech_to_text_google" speech_recognition_msgs::SpeechRecognitionCandidates))



(send *fetch* :torso_lift_joint :joint-angle 270)
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(600 250 680) :rpy (float-vector 0 0 pi/2))) :rotation-axis t :debug-view t)
(send *fetch* :head :look-at #f(500 250 500))
(send *ri* :angle-vector (send *fetch* :angle-vector) 7000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :move-end-pos #f(160 0 0))
(send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(600 250 680) :rpy (float-vector 0 0 pi/2))) :rotation-axis t :debug-view t)(send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :move-end-pos #f(160 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)


;;棚を開ける
(send *ri* :start-grasp :effort 60)
(unix:sleep 1)
(send *ri* :wait-interpolation)

(setq pull-range #f(150 0 0))

(send *fetch* :rarm :move-end-pos (v- #f(0 0 0) pull-range))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)

;;捜し物タイム
;;
;;

;;捜し物を終えるかの確認
;;
;;

(send *fetch* :rarm :move-end-pos pull-range)
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :effort 60)
(unix:sleep 1)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :move-end-pos #f(-160 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)


(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
(send *ri* :wait-interpolation)

(delete-shelf-collision-object)
