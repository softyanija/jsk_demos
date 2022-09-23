#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(load "package://assemble_table_pr2/euslisp/measure_socket_line_cam2.l")
(load "package://assemble_table_pr2/euslisp/adjust_pos_cam1.l")
(load "package://assemble_table_pr2/euslisp/adjust_pos_cam2.l")
(load "package://assemble_table_pr2/euslisp/adjust_angle_cam1.l")
(load "package://assemble_table_pr2/euslisp/adjust_angle_cam2.l")
(load "package://assemble_table_pr2/euslisp/pr2-manequine.l")
(pr2-init)
(objects (list *pr2*))

(move-to-init)
(measure-socket-line-cam2)
(pick-memory)
(adjust-angle-cam2)
(adjust-angle-cam1)

(adjust-cam2-pos-param)
(adjust-pos-cam2)

(adjust-cam1-pos-param)
(adjust-pos-cam1)

(set-memory)
(push-memory)


