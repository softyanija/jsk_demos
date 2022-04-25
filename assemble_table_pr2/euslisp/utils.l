(require "package://assemble_table_pr2/euslisp/parameters.l")

(defun screw-onestroke ()
  (send *pr2* :rarm :inverse-kinematics *rarm-pos-1*)
  (send *pr2* :larm :inverse-kinematics *larm-pos-1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm)
  (send *ri* :stop-grasp :larm)
  (unix:sleep 2)

  (send *pr2* :rarm :inverse-kinematics *rarm-pos-2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm)
  (unix:sleep 2)
  
  (send *ri* :stop-grasp :rarm)
  (unix:sleep 2)

  (send *pr2* :larm :inverse-kinematics *larm-pos-2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :rarm :inverse-kinematics *rarm-pos-1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm)
  (unix:sleep 2)

  (send *ri* :stop-grasp :larm)
  (unix:sleep 2)
  (send *pr2* :larm :inverse-kinematics *larm-pos-1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)  
  )
  
