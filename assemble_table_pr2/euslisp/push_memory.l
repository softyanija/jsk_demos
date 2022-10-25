#!/usr/bin/env roseus

(defun push_memory ()
  
  (larm-move-end-pos-worldcoords #f(0 0 18))
  (send *pr2* :larm :move-end-rot 90 :x)
  (send *pr2* :larm :move-end-rot -1 :z)
  (send *pr2* :larm :move-end-rot 1 :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  

  (larm-move-end-pos-worldcoords #f(0 45 0))
  (send *ri* :move-gripper :larm 0.030)

  (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) -20))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :move-end-rot -15 :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :move-end-rot 15 :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) 20))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (larm-move-end-pos-worldcoords #f(0 -87 0))

  (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) -20))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  (send *pr2* :larm :move-end-rot -15 :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :move-end-rot 15 :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) 20))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (larm-move-end-pos-worldcoords #f(0 -30 100))
