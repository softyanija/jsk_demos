(require "package://assemble_table_pr2/euslisp/parameters.l")

(defun screw-in-leg-pre ()
  (send *ri* :stop-grasp :rarm)
  (send *ri* :stop-grasp :larm)
  (send *pr2* :head_tilt_joint :joint-angle 35)
  
  (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(655 0 880) :rpy (float-vector 1.529 0.0 3.13)))
  (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(655 0 1000) :rpy (float-vector -0.0 -0.1 -0.0)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm)
  (send *ri* :stop-grasp :larm)

  (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(655 0 875) :rpy (float-vector 1.14 0.0 3.13)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm)
  (unix:sleep 1)
  
  (send *ri* :stop-grasp :rarm)
  (unix:sleep 1)

  (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(655 0 995) :rpy (float-vector -0.39 -0.1 -0.0)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(655 0 880) :rpy (float-vector 1.529 0.0 3.13)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm)
  (unix:sleep 1)

  (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(655 0 1000) :rpy (float-vector -0.0 -0.1 -0.0)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun screw-in-leg ()
  (send *ri* :stop-grasp :rarm)
  (send *ri* :stop-grasp :larm)
  (unix:sleep 1)
  (send *pr2* :head_tilt_joint :joint-angle 35)
  
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
  
