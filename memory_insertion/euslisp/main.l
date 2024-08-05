#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(load "package://assemble_table_pr2/euslisp/place_cams.l")
(load "package://assemble_table_pr2/euslisp/measure_socket_line_cam2.l")
(load "package://assemble_table_pr2/euslisp/adjust_pos_cam1.l")
(load "package://assemble_table_pr2/euslisp/adjust_pos_cam2.l")
(load "package://assemble_table_pr2/euslisp/adjust_angle_cam1.l")
(load "package://assemble_table_pr2/euslisp/adjust_angle_cam2.l")
(load "package://assemble_table_pr2/euslisp/pr2-manequine.l")
(load "package://assemble_table_pr2/euslisp/set_memory.l")
(load "package://assemble_table_pr2/euslisp/push_memory_improved.l")
(pr2-init)
(objects (list *pr2*))

(ros::roseus "third_eye")
(move-to-init)

(place-cam2)
(move-to-init)
(place-cam1)
(move-to-init)
(send *pr2* :angle-vector *rarm-down*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(measure-socket-line-cam2)
(pick-memory)
(adjust-angle-cam2)
(adjust-angle-cam1)

(get-cam1-pos-param)
(adjust-pos-cam1)

(get-cam2-pos-param)
(adjust-pos-cam2)

(adjust-pos-cam1)

(set-memory)

(push_memory)
