(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(objects (list *pr2*))
(send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(600 -200 850)))
(send *pr2* :larm :inverse-kinematics (make-coords :pos #f(600 200 850)))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :state :potentio-vector)

(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
