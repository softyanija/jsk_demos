(ros::roseus "hoge")
(ros::load-ros-manifest "speech_recognition_msgs")
(while (ros::ok)
  (setq message (one-shot-subscribe "/Tablet/voice" speech_recognition_msgs::SpeechRecognitionCandidates))
  (print (elt (send message :transcript) 0)))
  
