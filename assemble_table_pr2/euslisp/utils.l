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
  (send *ri* :stop-grasp :larm)
  (unix:sleep 1))

(defun pick-memory ()
  (send *pr2* :angle-vector *pick-memory-1*)
  (send *pr2* :larm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm)
  (unix:sleep 1)

  (send *pr2* :angle-vector *pick-memory-1*)
  (send *pr2* :larm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-3*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-4*)
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
  (setq *memory_cam1* (elt memory_cand max_i)))

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

