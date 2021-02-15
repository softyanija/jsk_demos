;;pull_shelf_3_imp.l
(ros::load-ros-manifest "std_msgs")

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
(send *ri* :wait-interpolation)

;;手を取っ手の位置まで持っていく
(send *fetch* :torso_lift_joint :joint-angle 270)
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(600 250 693) :rpy (float-vector 0 0 pi/2))) :rotation-axis t :debug-view t)
(send *fetch* :head :look-at #f(750 170 500))
(send *ri* :angle-vector (send *fetch* :angle-vector) 7000)
(send *ri* :wait-interpolation)

(ros::load-ros-manifest "std_msgs")
(setq f 0)
(setq dist 0)


(while (and (= f 0) (< dist 250))
  (setq pressure (one-shot-subscribe "/pressure" std_msgs::Float32))
  (if (> (send pressure :data) 4.0)
      (setq f 1)
    (progn
      (setq dist (+ dist 5))
      (send *fetch* :rarm :move-end-pos #f(5 0 0))
      (send *ri* :angle-vector (send *fetch* :angle-vector) 500)
      (send *ri* :wait-interpolation))))
  
#|
(send *fetch* :rarm :move-end-pos #f(180 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)
|#

;;取っ手をつかむ
(send *ri* :start-grasp :effort 60)
(unix:sleep 1)
(send *ri* :wait-interpolation)

(setq pull-range #f(150 0 0))

(send *fetch* :rarm :move-end-pos (v- #f(0 0 0) pull-range))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)

;;一旦放して肘を下げる
(send *ri* :stop-grasp :effort 60)
(unix:sleep 1)
(send *ri* :wait-interpolation)
(send *fetch* :rarm :move-end-pos #f(-50 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)
(send *fetch* :rarm :move-end-pos #f(0 -150 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)
