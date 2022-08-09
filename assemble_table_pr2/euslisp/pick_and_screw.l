(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(pr2-init)
(objects (list *pr2*))

(ros::roseus-add-msgs "geometry_msgs")
(ros::load-ros-manifest "geometry_msgs")

(setq *null-wrench* #f(0 0 0))
(setq push_z_lim 1.3)

(send *pr2* :angle-vector *pickup-leg-init*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 1)

(send *pr2* :larm :inverse-kinematics *larm-pickup-leg*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm)
(unix:sleep 1)

(send *pr2* :angle-vector *pickup-leg-init*)

(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :larm :inverse-kinematics *larm-pickup-leg2*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :larm :inverse-kinematics *larm-pickup-leg3*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-pickup-leg3*)
(send *pr2* :rarm :move-end-pos #f(-50 0 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-pickup-leg3*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(unix:sleep 1)

(send *ri* :stop-grasp :larm)
(unix:sleep 1)

(send *pr2* :larm :move-end-pos #f(-50 0 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :move-end-pos #f(0 -70 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(setq i 0)
(setq push_x_lim 1.3)
(while (< i 15)
  (setq force_x_p (send (send (send (one-shot-subscribe "/right_endeffector/wrench" geometry_msgs::WrenchStamped) :wrench) :force) :x))
  (setq force_x (* -1 force_x_p))
  (ros::ros-info (format nil "counter force is ~A" force_x))
  (if (> force_x push_x_lim)
      (progn
	(ros::ros-info "detect contacting")
	(return))
      (progn
	(ros::ros-info "still hanging")
	(rarm-move-end-pos-worldcoords #f(0 0 -3))

	(setq stroke (* 3 i))
	(setq i (+ i 1))
	(ros::ros-info (format nil "stroke is ~A" stroke)))))

(setq i 0)
(while (< i 5)
  (ros::ros-info (format nil "times : ~A" i))
  (rarm-screw-onestroke)
  (setq i (+ i 1)))
