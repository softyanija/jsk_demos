;; (load "package://baxtereus/baxter-interface.l")
;; (baxter-init)
;; (objects (list *baxter*))
;;(send *baxter* :angle-vector (send *ri* :state :potentio-vector))

(load "package://eus_vive/euslisp/lib/baxter-interface.l")
(load "package://assemble_table_baxter/euslisp/parameters.l")
(eus_vive::baxter-init :lgripper :softhand :rgripper :softhand)
(objects (list *baxter*))

(defun screw-onestroke ()
  (send *ri* :stop-grasp :larm)
  (unix:sleep 1)
  (send *baxter* :rarm :inverse-kinematics *rarm-pos-1*)
  (send *baxter* :larm :inverse-kinematics *larm-pos-1*)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm :rotate-angle (* pi 0))
  (unix:sleep 2)

  (send *baxter* :rarm :inverse-kinematics *rarm-pos-2*)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *baxter* :rarm :inverse-kinematics *rarm-pos-3*)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm :rotate-angle (* pi 0))
  (unix:sleep 2)
  (send *ri* :stop-grasp :rarm :rotate-angle (* pi -1))
  (unix:sleep 2)

  (send *baxter* :rarm :inverse-kinematics *rarm-pos-2*)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *baxter* :rarm :inverse-kinematics *rarm-pos-1*)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
  (send *ri* :wait-interpolation))

(setq i 1)
(do-until-key
 (ros::ros-info "times : ~A" i)
 (screw-onestroke)
 (setq i (+ i 1))
 (unix:sleep 1))
