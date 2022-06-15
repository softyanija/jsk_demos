;; (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
(load "package://eus_vive/euslisp/lib/baxter-interface.l")
(load "package://assemble_table_baxter/euslisp/parameters.l")
(eus_vive::baxter-init :lgripper :softhand :rgripper :softhand)
(objects (list *baxter*))

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 2)

(send *baxter* :rarm :inverse-kinematics *rarm-turnover-1*)
(send *baxter* :larm :inverse-kinematics *larm-turnover-1*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm :rotate-angle (* pi -0.3))
(send *ri* :start-grasp :larm :rotate-angle (* pi -0.3))
(unix:sleep 2)

(send *baxter* :rarm :inverse-kinematics *rarm-turnover-2*)
(send *baxter* :larm :inverse-kinematics *larm-turnover-2*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
(send *ri* :wait-interpolation)
