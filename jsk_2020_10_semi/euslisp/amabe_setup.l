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
