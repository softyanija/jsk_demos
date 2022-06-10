;; (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
(load "package://eus_vive/euslisp/lib/baxter-interface.l")
(load "package://assemble_table_baxter/euslisp/parameters.l")
(eus_vive::baxter-init :lgripper :softhand :rgripper :softhand)
(objects (list *baxter*))

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 2)

(defun screw-onestroke-vertical ()
  (send *baxter* :rarm :inverse-kinematics *rarm-screw-pos-1*)
  (send *baxter* :larm :inverse-kinematics *larm-screw-pos-1*)
  (send *baxter* :right_w2 :joint-angle -130)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm :rotate-angle (* pi -0.3))
  (unix:sleep 1)
  (send *ri* :stop-grasp :larm)
  (unix:sleep 1)

  (send *baxter* :right_w2 :joint-angle 0)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 2500)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm :rotate-angle (* pi 0))
  (unix:sleep 1)
  (send *ri* :stop-grasp :rarm :rotate-angle (* pi -1))
  (unix:sleep 2)
  
  (send *baxter* :rarm :move-end-pos #f(-50 0 0))
  (send *baxter* :right_w2 :joint-angle -130)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(send *baxter* :angle-vector *grasp_leg_init*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_pose*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm :rotate-angle (* pi 0.0))
(unix:sleep 2)

(send *baxter* :angle-vector *grasp_leg_init*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm :rotate-angle (* pi -0.3))
(unix:sleep 2)

(send *baxter* :angle-vector *grasp_leg_inter1*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2500)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_inter2*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2500)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_inter3*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2500)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_inter4*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_top_pre*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_top*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2500)
(send *ri* :wait-interpolation)

(screw-onestroke-vertical)

;; (setq i 1)
;; (do-until-key
;;  (ros::ros-info "times : ~A" i)
;;  (screw-onestroke-vertical)
;;  (setq i (+ i 1))
;;  (unix:sleep 1))
