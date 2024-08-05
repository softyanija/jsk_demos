;;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(objects (list *pr2*))

(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 1)

(send *pr2* :l_wrist_roll_joint :joint-angle (+ (send *pr2* :l_wrist_roll_joint :joint-angle) 180))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)


(setq i 0)
(while (< i 20)
  (ros::ros-info (format nil "times : ~A" i))
  (screw-onestroke)
  (setq i (+ i 1)))
