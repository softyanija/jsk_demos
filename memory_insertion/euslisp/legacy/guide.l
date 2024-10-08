;;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(pr2-init)
(objects (list *pr2*))

(ros::roseus-add-msgs "geometry_msgs")
(ros::load-ros-manifest "geometry_msgs")

(setq *null-wrench* #f(0 0 0))
(setq push_z_lim 1.5)

(send *pr2* :reset-manip-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 1)

(send *ri* :move-gripper :rarm 0.020)
(unix:sleep 1)

(send *pr2* :rarm :inverse-kinematics *rarm-pre-guide*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :larm :inverse-kinematics *larm-pre-guide*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm)
(unix:sleep 1)

(send *pr2* :larm :inverse-kinematics *larm-guide*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

;;roop
(setq stroke (transform (inverse-matrix (send (send *pr2* :larm :end-coords) :worldrot)) #f(0 0 -10)))
(send *pr2* :larm :move-end-pos stroke)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(setq stroke (transform (inverse-matrix (send (send *pr2* :larm :end-coords) :worldrot)) #f(-3 0 0)))
(send *pr2* :larm :move-end-pos stroke)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)
;; e(transform (inverse-matrix (send (send *pr2* :larm :end-coords) :worldrot)) #f(0 0 -10))
(send 
;; (setq i 0)
;; (while (< i 15)
;;   (setq force_z_p (send (send (send (one-shot-subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped) :wrench) :force) :z))
;;   (setq force_z (* -1 force_z_p))
;;   (ros::ros-info (format nil "counter force is ~A" force_z))
;;   (if (> force_z push_z_lim)
;;       (progn
;; 	(ros::ros-info "detect contacting")
;; 	(return))
;;       (progn
;; 	(ros::ros-info "still hanging")
;; 	(send *pr2* :larm :move-end-pos #f(0 0 3))
;; 	(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
;; 	(send *ri* :wait-interpolation)

;; 	(setq stroke (* 3 i))
;; 	(setq i (+ i 1))
;; 	(ros::ros-info (format nil "stroke is ~A" stroke)))))

;;#f(1 0 0) or #f(-1 0 0)

