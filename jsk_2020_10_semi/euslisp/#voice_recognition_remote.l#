(defun voice_which()
  (setq f_w 0)
  (while (and (ros::ok) (= f_w 0))
    (setq message (one-shot-subscribe "/Tablet/voice" speech_recognition_msgs::SpeechRecognitionCandidates))
    (setq message (elt (send message :transcript) 0))
    (print message)
    (cond ((or (string= message "3段目") (string= message "三段目") (string= message "3番目")) (progn
												 (setq f_w 3)
												 ))
	  ((or (string= message "2段目") (string= message "二段目") (string= message "2番目")) (progn
												 (setq f_w 2)
												 ))
	  ((or (string= message "4段目") (string= message "四段目") (string= message "4番目")) (progn
												 (setq f_w 4)
												 ))
	  (t)))
  f_w)
