;;pull_shelf_4.l
(ros::load-ros-manifest "std_msgs")

(add-shelf-collision-object)
(add-left-collision-object)

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
(send *ri* :wait-interpolation)

;;手を取っ手の位置まで持っていく
(send *fetch* :torso_lift_joint :joint-angle 130)
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(600 250 511) :rpy (float-vector 0 0 pi/2))) :rotation-axis t :debug-view t)
(send *ri* :angle-vector (send *fetch* :angle-vector) 7000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :move-end-pos #f(100 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
(send *ri* :wait-interpolation)

;;手が棚に当たるまで前に出す
(ros::load-ros-manifest "std_msgs")
(setq f 0)
(setq dist 0)

(delete-shelf-collision-object)

(while (= f 0)
  (while (and (= f 0) (< dist 250))
    (setq pressure (one-shot-subscribe "/pressure" std_msgs::Float32))
    (if (> (send pressure :data) 4.0)
	(setq f 1)
      (progn
	(setq dist (+ dist 8))
	(send *fetch* :rarm :move-end-pos #f(8 0 0))
	(send *ri* :angle-vector (send *fetch* :angle-vector) 200 :use-torso nil)
	(send *ri* :wait-interpolation)
	(ros::ros-info (format nil "dist : ~A" dist)))))
  (if (= f 0)
      (progn
	(send *fetch* :rarm :move-end-pos (float-vector (- 0 dist) 0 0))
	(send *ri* :angle-vector (send *fetch* :angle-vector) 1000 :use-torso nil)
	(send *ri* :wait-interpolation)
	(send *ri* :go-POS-UNSAFE 0.1 0 0)
	(setq dist 0))))

;;顔を手の方へ向ける
(send *fetch* :head :look-at (float-vector (+ 670 dist) 170 350))
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000)

;;取っ手をつかむ
(send *ri* :start-grasp :effort 60)
(unix:sleep 1)
(send *ri* :wait-interpolation)

(setq pull-range (float-vector (+ dist 70) 0 0))

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
