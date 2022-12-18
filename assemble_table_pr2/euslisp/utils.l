(ros::roseus-add-msgs "opencv_apps")
(ros::roseus-add-msgs "geometry_msgs")

(ros::load-ros-manifest "opencv_apps")
(ros::load-ros-manifest "geometry_msgs")

(defun reflect ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

(defun manequine-both ()
  (manequine-mode :arm (list "rarm" "larm")))

(defun rarm-move-end-pos-worldcoords (stroke-worldcoords)
  (setq stroke-rarm (transform (inverse-matrix (send (send *pr2* :rarm :end-coords) :worldrot)) stroke-worldcoords))
  (send *pr2* :rarm :move-end-pos stroke-rarm)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation))

(defun larm-move-end-pos-worldcoords (stroke-worldcoords)
  (setq stroke-larm (transform (inverse-matrix (send (send *pr2* :larm :end-coords) :worldrot)) stroke-worldcoords))
  (send *pr2* :larm :move-end-pos stroke-larm)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation))

(defun get-pos ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (ros::ros-info (format nil "rarm-corods is ~A" (send *pr2* :rarm :end-coords)))
  (ros::ros-info (format nil "larm-coords is ~A" (send *pr2* :larm :end-coords))))

(defun get-angle-vector ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

(defun move-to-init ()
  (send *pr2* :angle-vector *init-pose*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :larm)
  (send *ri* :stop-grasp :rarm)
  (unix:sleep 1))

(defun pick-memory ()
  (send *pr2* :angle-vector *pick-memory-1*)
  (send *pr2* :rarm :inverse-kinematics (send (send *pr2* :copy-worldcoords) :transform (make-coords :pos #f(764.215 -305.59 831.716) :rpy (float-vector 0.357 0.162 0.032))) :translation-axis t :rotation-axis t)
  (send *pr2* :larm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-1*)
  (send *pr2* :rarm :inverse-kinematics (send (send *pr2* :copy-worldcoords) :transform (make-coords :pos #f(764.215 -305.59 831.716) :rpy (float-vector 0.357 0.162 0.032))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :move-gripper :larm 0.002 :effort 60)
  (unix:sleep 1)

  (send *pr2* :angle-vector *pick-memory-1*)
  (send *pr2* :rarm :inverse-kinematics (send (send *pr2* :copy-worldcoords) :transform (make-coords :pos #f(764.215 -305.59 831.716) :rpy (float-vector 0.357 0.162 0.032))) :translation-axis t :rotation-axis t)
  (send *pr2* :larm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-2*)
  (send *pr2* :rarm :inverse-kinematics (send (send *pr2* :copy-worldcoords) :transform (make-coords :pos #f(764.215 -305.59 831.716) :rpy (float-vector 0.357 0.162 0.032))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-3*)
  (send *pr2* :rarm :inverse-kinematics (send (send *pr2* :copy-worldcoords) :transform (make-coords :pos #f(764.215 -305.59 831.716) :rpy (float-vector 0.357 0.162 0.032))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-4*)
  (send *pr2* :rarm :inverse-kinematics (send (send *pr2* :copy-worldcoords) :transform (make-coords :pos #f(764.215 -305.59 831.716) :rpy (float-vector 0.357 0.162 0.032))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun get-cam1-memory ()
  (setq f 0)
  (while (= f 0)
    (setq memory_cand (send (one-shot-subscribe "/timer_cam1_rec/memory/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
    (when memory_cand
      (progn
        (setq f 1)
        (setq l (length memory_cand))
        (setq i 0)
        (setq size_max 0)
        (setq lim_y 50)
        (while (< i l)
          (setq rect (elt memory_cand i))
          (if (< lim_y (send (send rect :center) :y))
              (progn
                (setq size_buf (* (send (send rect :size) :width) (send (send rect :size) :height)))
                (if (> size_buf size_max)
                    (progn
                      (setq size_max size_buf)
                      (setq max_i i)))))
          (incf i)))))
  (setq *memory_cam1* (elt memory_cand max_i))
  (ros::ros-info (format nil "x is ~A, y is ~A" (send (send *memory_cam1* :center) :x) (send (send *memory_cam1* :center) :y))))

(defun get-cam2-memory-edge ()
  (setq f 0)
  (while (= f 0)
    (setq memory_edge_cand (send (one-shot-subscribe "/timer_cam2_rec/memory_edge" geometry_msgs::PoseArray) :poses))
    (when memory_edge_cand
      (progn
        (setq f 1)
        (setq l (length memory_edge_cand))
        (setq i 0)
        (setq max_sum 0)
        (setq max_i 0)
        (while (< i l)
          (setq x (send (send (elt memory_edge_cand i) :position) :x))
          (setq y (send (send (elt memory_edge_cand i) :position) :y))
          (setq buf_sum (+ x y))
          (if (> buf_sum max_sum)
              (progn
                (setq max_sum buf_sum)
                (setq max_i i)))
          (incf i))
        (setq *memory_edge_cam2* (send (elt memory_edge_cand max_i) :position))))))

(defun get-cam2-socket ()
  (setq f 0)
  (while (= f 0)
    (setq socket_cand (send (one-shot-subscribe "/timer_cam2_rec/socket/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
    (when socket_cand
      (progn
        (setq l (length socket_cand))
        (setq i 0)
        (setq size_max 0)
        (while (< i l)
          (setq rect (elt socket_cand i))
          (setq size_buf (* (send (send rect :size) :width) (send (send rect :size) :height)))
          (if (> size_buf size_max)
              (progn
                (setq size_max size_buf)
                (setq max_i i)))
          (incf i))
        (setq socket (elt socket_cand max_i))
        (if (> size_max 100)
            (progn
              (setq f 1)
              (setq *socket_cam2* socket)))
        ))))

(defun check-set-memory-cam2 ()
  (get-cam2-memory-edge)
  (get-cam2-socket)
  (setq th 3)
  (setq memory_edge (send *memory_edge_cam2* :y))
  (setq socket_center (send (send *socket_cam2* :center) :y))
  (> (+ memory_edge th) socket_center))

(defun judge-set-memory-cam2 ()
  (get-cam2-memory-edge)
  (get-cam2-socket)
  (setq socket_before_x (send (send *socket_cam2* :center) :x))
  (setq socket_before_y (send (send *socket_cam2* :center) :y))

  (setq result 0)
  (setq socket_cand (send (one-shot-subscribe "/timer_cam2_rec/socket/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
  (setq size_max 0)
  (when socket_cand
    (progn
      (setq l (length socket_cand))
      (setq i 0)
      (setq size_max 0)
      (while (< i l)
        (setq rect (elt socket_cand i))
        (setq size_buf (* (send (send rect :size) :width) (send (send rect :size) :height)))
        (if (> size_buf size_max)
            (progn
              (setq size_max size_buf)
              (setq max_i i)))
        (incf i))
      (setq socket_buf (elt socket_cand max_i))
      (if (< size_max 100)
          (progn
            (ros::ros-info "memory is front side")
            2)
          (progn
              (setq f 4)
              (setq memory_x (send *memory_edge_cam2* :x))
              (setq memory_y (send *memory_edge_cam2* :y))
              (setq socket_x (send (send socket_buf :center) :x))
              (setq socket_y (send (send socket_buf :center) :y))
              (setq diff_x (- memory_x socket_x))
              (setq diff_y (- memory_y socket_y))
              (cond ((> diff_y 1) (progn
                                      (ros::ros-info "memory is set")
                                      0))
                    ((> diff_y -4) (progn
                                     (ros::ros-info "memory is rear")
                                     1))
                    (t (progn
                         (ros::ros-info "memory is on the socket edge")
                         3))
                    )
          )))))
