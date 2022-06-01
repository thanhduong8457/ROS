;; Auto-generated. Do not edit!


(when (boundp 'my_delta_robot::angulo)
  (if (not (find-package "MY_DELTA_ROBOT"))
    (make-package "MY_DELTA_ROBOT"))
  (shadow 'angulo (find-package "MY_DELTA_ROBOT")))
(unless (find-package "MY_DELTA_ROBOT::ANGULO")
  (make-package "MY_DELTA_ROBOT::ANGULO"))

(in-package "ROS")
;;//! \htmlinclude angulo.msg.html


(defclass my_delta_robot::angulo
  :super ros::object
  :slots (_theta1 _theta2 _theta3 ))

(defmethod my_delta_robot::angulo
  (:init
   (&key
    ((:theta1 __theta1) 0.0)
    ((:theta2 __theta2) 0.0)
    ((:theta3 __theta3) 0.0)
    )
   (send-super :init)
   (setq _theta1 (float __theta1))
   (setq _theta2 (float __theta2))
   (setq _theta3 (float __theta3))
   self)
  (:theta1
   (&optional __theta1)
   (if __theta1 (setq _theta1 __theta1)) _theta1)
  (:theta2
   (&optional __theta2)
   (if __theta2 (setq _theta2 __theta2)) _theta2)
  (:theta3
   (&optional __theta3)
   (if __theta3 (setq _theta3 __theta3)) _theta3)
  (:serialization-length
   ()
   (+
    ;; float32 _theta1
    4
    ;; float32 _theta2
    4
    ;; float32 _theta3
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _theta1
       (sys::poke _theta1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta2
       (sys::poke _theta2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta3
       (sys::poke _theta3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _theta1
     (setq _theta1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta2
     (setq _theta2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta3
     (setq _theta3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get my_delta_robot::angulo :md5sum-) "f6b72178218fbe6abc22ec5b1c7a40d8")
(setf (get my_delta_robot::angulo :datatype-) "my_delta_robot/angulo")
(setf (get my_delta_robot::angulo :definition-)
      "float32 theta1
float32 theta2
float32 theta3

")



(provide :my_delta_robot/angulo "f6b72178218fbe6abc22ec5b1c7a40d8")


