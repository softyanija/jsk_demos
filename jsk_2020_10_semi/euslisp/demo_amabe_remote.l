#!/usr/bin/env roseus

(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(ros::roseus-add-msgs "std_msgs")

(ros::load-ros-manifest "speech_recognition_msgs")

(ros::roseus "amabe")

(load "voice_recognition_remote.l")
(load "voice_confirm_remote.l")

(unless (boundp '*scene*)
 (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
 (fetch-init))

;;棚を障害物として設定
(defun add-shelf-collision-object ()
  (let ((shelf (make-cube 10 1000 1100)))
    (send shelf :translate #f(850 0 550))
    (send *co* :add-object shelf :frame-id "base_link" :object-id "shelf")))

;;それを削除
(defun delete-shelf-collision-object ()
  (send *co* :delete-object-by-id "shelf"))

;;左側に障害物を設定
(defun add-left-collision-object ()
  (let ((shelf (make-cube 1500 10 1100)))
    (send shelf :translate #f(0 650 550))
    (send *co* :add-object shelf :frame-id "base_link" :object-id "left")))

;;左側の障害物を削除
(defun delete-left-collision-object ()
  (send *co* :delete-object-by-id "left"))

;;Realsence回避用の障害物を設定
#|
(defun add-realsence-collision-object ()
  (let ((keepout (make-cylinder 300 50)))
    (send keepout :translate #f(0 500 500))
    (send *co* :add-object keepout :frame-id "base-link" :object-id "realsence")))
|#
(defun add-realsence-collision-object ()
  (let ((cube (make-cube 450 400 50))
          (keepout (make-cylinder 350 50)))
      (send cube :translate #f(-30 0 0))
      (send keepout :translate #f(0 0 -25))
      (setq keepout (body- keepout cube))
      (send keepout :translate #f(0 0 400))
      (send *co* :add-object keepout :frame-id "base_link" :object-id "realsence")))

;;Realsence回避用の障害物を削除
(defun delete-realsence-collision-object ()
  (send *co* :delete-object-by-id "realsence"))

(defun pull_shelf_2()
  (format t ";;pull_shelf_2")
  (load "pull_shelf_2.l")
  )

(defun return_shelf_2()
  (format t ";;return_shelf_2")
  (load "return_shelf_2.l")
  )

(defun pull_shelf_3()
  (format t ";;pull_shelf_3")
  (load "pull_shelf_3.l")
  )

(defun return_shelf_3()
  (format t ";;return_shelf_3")
  (load "return_shelf_3.l")
  )

(defun pull_shelf_4()
  (format t ";;pull_shelf_4")
  (load "pull_shelf_4.l")
  )

(defun return_shelf_4()
  (format t ";;return_shelf_4")
  (load "return_shelf_4.l")
  )

(defun demo_amabe_remote()
  (let ((an (instance std_msgs::UInt16 :init)))
    (ros::advertise "/announce" std_msgs::UInt16 1)
    (unix:sleep 1)
    (send an :data 0)
    (ros::publish "/announce" an)
    (ros::spin-once)
    (print "start demo_amabe")
    (setq an 1)
    (setq a (voice_which))
    (print "start voice_r")
    (cond ((= a 2) (progn
		     (setq an 2)
		     (pull_shelf_2)
		     (setq an 5)
		     (voice_confirm)
		     (setq an 6)
		     (return_shelf_2)
		     ))
	  ((= a 3) (progn
		     (setq an 3)
		     (pull_shelf_3)
		     (setq an 5)
		     (voice_confirm)
		     (setq an 6)
		     (return_shelf_3)
		     ))
	  ((= a 4) (progn
		     (setq an 4)
		     (pull_shelf_4)
		     (setq an 5)
		     (voice_confirm)
		     (setq an 6)
		     (return_shelf_4)
		     ))
	  (t (print "エラー")))
    (setq an 7)
    ))


(print "start")

;;(add-shelf-collision-object)

(demo_amabe_remote)

;;(delete-shelf-collision-object)

(print "finish")
