(defun screw-in-leg ()
  (send *ri* :stop-grasp :rarm)
  (send *ri* :stop-grasp :larm)
  
  (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(655 0 880) :rpy (float-vector 1.529 0.0 3.13)))
  (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(655 0 1000) :pry (float-vector -0.352 -0.152 -0.098)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm)
  (send *ri* :stop-grasp :larm)

  (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(655 0 876) :rpy (float-vector 0.81 0.0 3.13)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm))
  
  
