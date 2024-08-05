#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(load "package://assemble_table_pr2/euslisp/pr2-manequine.l")

(pr2-init)
(objects (list *pr2*))

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::load-ros-manifest "opencv_apps")

(ros::roseus "memory_insertion_test")

;;set to init pose
(send *pr2* :angle-vector #f(286.667 48.3234 11.8444 49.2446 -86.3385 -241.919 -96.8036 368.248 6.71635 -12.8288 -107.604 -58.0285 -70.5486 -83.4179 -211.503 6.2287 -21.2682))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;;grasp memory
(send *ri* :start-grasp :rarm)

(send *pr2* :rarm :move-end-pos #f(180 0 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;;adjust
(send *pr2* :rarm :move-end-pos #f(0 1 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :move-end-pos #f(0 -1 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)
;;intertion


(send *ri* :move-gripper :rarm 0.020)

(send *pr2* :rarm :move-end-pos #f(-50 0 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(unix:sleep 1)

(send *pr2* :r_wrist_roll_joint :joint-angle (+ (send *pr2* :r_wrist_roll_joint :joint-angle) 90))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) -10))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)
;;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
