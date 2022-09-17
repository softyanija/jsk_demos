;;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(load "package://assemble_table_pr2/euslisp/pr2-manequine.l")
(pr2-init)
(objects (list *pr2*))

(ros::roseus-add-msgs "geometry_msgs")
(ros::load-ros-manifest "geometry_msgs")

(setq *null-wrench* #f(0 0 0))
(setq push_z_lim 1.3)

(send *pr2* :reset-manip-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 1)

(send *pr2* :larm :inverse-kinematics *larm-pre-apply*)
(send *pr2* :rarm :inverse-kinematics *rarm-pre-apply*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :move-gripper :rarm 0.020)
(unix:sleep 1)

(send *ri* :start-grasp :larm)
(unix:sleep 1)

(setq i 0)
(setq push_z_lim 1.3)
(while (< i 15)
  (setq force_z_p (send (send (send (one-shot-subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped) :wrench) :force) :z))
  (setq force_z (* 1 force_z_p))
  (ros::ros-info (format nil "counter force is ~A" force_z))
  (if (> force_z push_z_lim)
      (progn
	(ros::ros-info "detect contacting")
	(return))
      (progn
	(ros::ros-info "still hanging")
	(larm-move-end-pos-worldcoords #f(0 0 -3))

	(setq stroke (* 3 i))
	(setq i (+ i 1))
	(ros::ros-info (format nil "stroke is ~A" stroke)))))

;;chech if botl and nat is -- directly
(send *pr2* :rarm :inverse-kinematics *rarm-apply*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-apply2*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-pre-apply*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)

(send *pr2* :rarm :inverse-kinematics *rarm-before-rotation1*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-before-rotation2*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-before-rotation3*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(unix:sleep 1)

(send *ri* :stop-grasp :larm)
(unix:sleep 1)

(send *pr2* :larm :move-end-pos #f(-60 0 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;try to screw-in and check if insertion was successed
(setq i 0)
(while (< i 5)
  (ros::ros-info (format nil "times : ~A" i))
  (rarm-screw-onestroke)
  (setq i (+ i 1)))


#|
(setq j 0)
(while (< j 15)
  (ros::ros-info (format nil "j : ~A" j)) 
  (rarm-move-end-pos-worldcoords #f(0 2 0))
  (setq j (+ j 1)))

(send *pr2* :rarm :move-end-pos #f(3 0 3))
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)

(load "models/arrow-object.l") 
(arrow)
(send a :newcoords (send (send *pr2* :rarm :end-coords) :worldcoords))
|#
