(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(load "package://assemble_table_pr2/euslisp/pr2-manequine.l")
(pr2-init)
(objects (list *pr2*))

(move-to-init)
(pick-memory)


