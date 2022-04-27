;; (load "package://baxtereus/baxter-interface.l")
;; (baxter-init)
;; (objects (list *baxter*))

(load "package://eus_vive/euslisp/lib/baxter-interface.l")
(eus_vive::baxter-init :lgripper :softhand :rgripper :softhand)
(objects (list *baxter*))
