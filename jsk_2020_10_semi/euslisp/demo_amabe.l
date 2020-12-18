#!/usr/bin/env roseus

(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(ros::roseus "amabe")
(ros::load-ros-manifest "speech_recognition_msgs")

(load "voice_recognition.l")
(load "voice_confirm.l")

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

(defun pull_shelf_2()
  (format t ";;pull_shelf_2")
  (load "pull_shelf_2.l")
  )

(defun return_shelf_2()
  (format t ";;return_shelf_2")
  (load "return_shelf_2.l")
  )

(defun demo_amabe()
  (print "start demo_amabe")
  (setq a (voice_which))
  (print "start voice_r")
  (cond ((= a 3) (progn
		   (pull_shelf_2)
		   (voice_confirm)
		   (return_shelf_2)
		   ))
	(t (print "エラー")))
  )


(print "start")

(add-shelf-collision-object)

(demo_amabe)

(delete-shelf-collision-object)

(print "finish")
