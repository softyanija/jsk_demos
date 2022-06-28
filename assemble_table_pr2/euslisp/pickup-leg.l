;;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(pr2-init)
(objects (list *pr2*))

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
