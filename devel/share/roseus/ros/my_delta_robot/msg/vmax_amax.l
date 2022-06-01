;; Auto-generated. Do not edit!


(when (boundp 'my_delta_robot::vmax_amax)
  (if (not (find-package "MY_DELTA_ROBOT"))
    (make-package "MY_DELTA_ROBOT"))
  (shadow 'vmax_amax (find-package "MY_DELTA_ROBOT")))
(unless (find-package "MY_DELTA_ROBOT::VMAX_AMAX")
  (make-package "MY_DELTA_ROBOT::VMAX_AMAX"))

(in-package "ROS")
;;//! \htmlinclude vmax_amax.msg.html


(defclass my_delta_robot::vmax_amax
  :super ros::object
  :slots (_vmax _amax ))

(defmethod my_delta_robot::vmax_amax
  (:init
   (&key
    ((:vmax __vmax) 0.0)
    ((:amax __amax) 0.0)
    )
   (send-super :init)
   (setq _vmax (float __vmax))
   (setq _amax (float __amax))
   self)
  (:vmax
   (&optional __vmax)
   (if __vmax (setq _vmax __vmax)) _vmax)
  (:amax
   (&optional __amax)
   (if __amax (setq _amax __amax)) _amax)
  (:serialization-length
   ()
   (+
    ;; float64 _vmax
    8
    ;; float64 _amax
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _vmax
       (sys::poke _vmax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _amax
       (sys::poke _amax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _vmax
     (setq _vmax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _amax
     (setq _amax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get my_delta_robot::vmax_amax :md5sum-) "2ab9faecee53d06a238561f5c5a17a83")
(setf (get my_delta_robot::vmax_amax :datatype-) "my_delta_robot/vmax_amax")
(setf (get my_delta_robot::vmax_amax :definition-)
      "float64 vmax
float64 amax
")



(provide :my_delta_robot/vmax_amax "2ab9faecee53d06a238561f5c5a17a83")


