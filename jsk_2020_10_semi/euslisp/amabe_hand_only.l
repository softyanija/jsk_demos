#!/usr/bin/env roseus

(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")


(unless (boundp '*scene*)
  (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
  (fetch-init))
#|棚に干渉しないようにする
(:add-ground-collision-object ()
    (let ((cube (make-cube 200 350 50))
          (ground (make-cylinder 1000 50)))
      (send cube :translate #f(-120 0 0))
      (send ground :translate #f(0 0 -25))
      (setq ground (body- ground cube))
      (send ground :translate #f(0 0 -25))
      (send *co* :add-object ground :frame-id base-frame-id :object-id "ground")))
  (:delete-ground-collision-object ()
    (send *co* :delete-object-by-id "ground")))
|#

#|棚を障害物として設定
(defun add-shelf-collision-object ()
  (let ((shelf (make-cube 10 1000 1100)))
    (send shelf :translate #f(0 795 550))
    (send *co* :add-object shelf :frame-id "base-link" :object-id "shelf")))

(defun delete-shelf-collision-object ()
  (send *co* :delete-object-by-id "shelf"))
|#

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(600 250 680) :rpy (float-vector 0 0 pi/2))) :rotation-axis t :debug-view t)(send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :move-end-pos #f(210 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)


;;open shelf 
(send *ri* :start-grasp :effort 60)
(unix:sleep 1)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :move-end-pos #f(-210 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)
(send *fetch* :rarm :move-end-pos #f(210 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)


(send *ri* :stop-grasp :effort 60)
(unix:sleep 1)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :move-end-pos #f(-210 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
(send *ri* :wait-interpolation)
