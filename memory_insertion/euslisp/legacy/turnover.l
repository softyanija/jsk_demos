;;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(pr2-init)
(objects (list *pr2*))

(send *pr2* :reset-manip-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 1)

(send *pr2* :rarm :inverse-kinematics *rarm-pre-grasp-table-pos*)
(send *pr2* :larm :inverse-kinematics *larm-pre-grasp-table-pos*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-grasp-table-pos*)
(send *pr2* :larm :inverse-kinematics *larm-grasp-table-pos*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(send *ri* :start-grasp :larm)
(unix:sleep 1)

(send *pr2* :rarm :inverse-kinematics *rarm-pickup-table-pos1*)
(send *pr2* :larm :inverse-kinematics *larm-pickup-table-pos1*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-pickup-table-pos2*)
(send *pr2* :larm :inverse-kinematics *larm-pickup-table-pos2*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-pickup-table-pos3*)
(send *pr2* :larm :inverse-kinematics *larm-pickup-table-pos3*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 1)

(send *pr2* :rarm :inverse-kinematics *rarm-pickup-table-pos4*)
(send *pr2* :larm :inverse-kinematics *larm-pickup-table-pos4*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-pickup-table-pos5*)
(send *pr2* :larm :inverse-kinematics *larm-pickup-table-pos5*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-pickup-table-pos6*)
(send *pr2* :larm :inverse-kinematics *larm-pickup-table-pos6*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(send *ri* :start-grasp :larm)
(unix:sleep 2)

(send *pr2* :rarm :inverse-kinematics *rarm-pickup-table-pos7*)
(send *pr2* :larm :inverse-kinematics *larm-pickup-table-pos7*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *rarm-pickup-table-pos8*)
(send *pr2* :larm :inverse-kinematics *larm-pickup-table-pos8*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 2)

(send *pr2* :rarm :move-end-pos #f(-80 0 0))
(send *pr2* :larm :move-end-pos #f(-80 0 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)
