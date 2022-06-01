;; Auto-generated. Do not edit!


(when (boundp 'my_delta_robot::PositionArm)
  (if (not (find-package "MY_DELTA_ROBOT"))
    (make-package "MY_DELTA_ROBOT"))
  (shadow 'PositionArm (find-package "MY_DELTA_ROBOT")))
(unless (find-package "MY_DELTA_ROBOT::POSITIONARM")
  (make-package "MY_DELTA_ROBOT::POSITIONARM"))

(in-package "ROS")
;;//! \htmlinclude PositionArm.msg.html


(defclass my_delta_robot::PositionArm
  :super ros::object
  :slots (_base_brazo1 _base_brazo2 _base_brazo3 _codo1_a _codo1_b _codo2_a _codo2_b _codo3_a _codo3_b _act_x _act_y _act_z ))

(defmethod my_delta_robot::PositionArm
  (:init
   (&key
    ((:base_brazo1 __base_brazo1) 0.0)
    ((:base_brazo2 __base_brazo2) 0.0)
    ((:base_brazo3 __base_brazo3) 0.0)
    ((:codo1_a __codo1_a) 0.0)
    ((:codo1_b __codo1_b) 0.0)
    ((:codo2_a __codo2_a) 0.0)
    ((:codo2_b __codo2_b) 0.0)
    ((:codo3_a __codo3_a) 0.0)
    ((:codo3_b __codo3_b) 0.0)
    ((:act_x __act_x) 0.0)
    ((:act_y __act_y) 0.0)
    ((:act_z __act_z) 0.0)
    )
   (send-super :init)
   (setq _base_brazo1 (float __base_brazo1))
   (setq _base_brazo2 (float __base_brazo2))
   (setq _base_brazo3 (float __base_brazo3))
   (setq _codo1_a (float __codo1_a))
   (setq _codo1_b (float __codo1_b))
   (setq _codo2_a (float __codo2_a))
   (setq _codo2_b (float __codo2_b))
   (setq _codo3_a (float __codo3_a))
   (setq _codo3_b (float __codo3_b))
   (setq _act_x (float __act_x))
   (setq _act_y (float __act_y))
   (setq _act_z (float __act_z))
   self)
  (:base_brazo1
   (&optional __base_brazo1)
   (if __base_brazo1 (setq _base_brazo1 __base_brazo1)) _base_brazo1)
  (:base_brazo2
   (&optional __base_brazo2)
   (if __base_brazo2 (setq _base_brazo2 __base_brazo2)) _base_brazo2)
  (:base_brazo3
   (&optional __base_brazo3)
   (if __base_brazo3 (setq _base_brazo3 __base_brazo3)) _base_brazo3)
  (:codo1_a
   (&optional __codo1_a)
   (if __codo1_a (setq _codo1_a __codo1_a)) _codo1_a)
  (:codo1_b
   (&optional __codo1_b)
   (if __codo1_b (setq _codo1_b __codo1_b)) _codo1_b)
  (:codo2_a
   (&optional __codo2_a)
   (if __codo2_a (setq _codo2_a __codo2_a)) _codo2_a)
  (:codo2_b
   (&optional __codo2_b)
   (if __codo2_b (setq _codo2_b __codo2_b)) _codo2_b)
  (:codo3_a
   (&optional __codo3_a)
   (if __codo3_a (setq _codo3_a __codo3_a)) _codo3_a)
  (:codo3_b
   (&optional __codo3_b)
   (if __codo3_b (setq _codo3_b __codo3_b)) _codo3_b)
  (:act_x
   (&optional __act_x)
   (if __act_x (setq _act_x __act_x)) _act_x)
  (:act_y
   (&optional __act_y)
   (if __act_y (setq _act_y __act_y)) _act_y)
  (:act_z
   (&optional __act_z)
   (if __act_z (setq _act_z __act_z)) _act_z)
  (:serialization-length
   ()
   (+
    ;; float64 _base_brazo1
    8
    ;; float64 _base_brazo2
    8
    ;; float64 _base_brazo3
    8
    ;; float64 _codo1_a
    8
    ;; float64 _codo1_b
    8
    ;; float64 _codo2_a
    8
    ;; float64 _codo2_b
    8
    ;; float64 _codo3_a
    8
    ;; float64 _codo3_b
    8
    ;; float64 _act_x
    8
    ;; float64 _act_y
    8
    ;; float64 _act_z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _base_brazo1
       (sys::poke _base_brazo1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _base_brazo2
       (sys::poke _base_brazo2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _base_brazo3
       (sys::poke _base_brazo3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _codo1_a
       (sys::poke _codo1_a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _codo1_b
       (sys::poke _codo1_b (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _codo2_a
       (sys::poke _codo2_a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _codo2_b
       (sys::poke _codo2_b (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _codo3_a
       (sys::poke _codo3_a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _codo3_b
       (sys::poke _codo3_b (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _act_x
       (sys::poke _act_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _act_y
       (sys::poke _act_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _act_z
       (sys::poke _act_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _base_brazo1
     (setq _base_brazo1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _base_brazo2
     (setq _base_brazo2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _base_brazo3
     (setq _base_brazo3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _codo1_a
     (setq _codo1_a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _codo1_b
     (setq _codo1_b (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _codo2_a
     (setq _codo2_a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _codo2_b
     (setq _codo2_b (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _codo3_a
     (setq _codo3_a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _codo3_b
     (setq _codo3_b (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _act_x
     (setq _act_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _act_y
     (setq _act_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _act_z
     (setq _act_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get my_delta_robot::PositionArm :md5sum-) "d2558a79329645ec1605ebea813b04a6")
(setf (get my_delta_robot::PositionArm :datatype-) "my_delta_robot/PositionArm")
(setf (get my_delta_robot::PositionArm :definition-)
      "float64 base_brazo1
float64 base_brazo2
float64 base_brazo3
float64 codo1_a
float64 codo1_b
float64 codo2_a
float64 codo2_b
float64 codo3_a
float64 codo3_b
float64 act_x
float64 act_y
float64 act_z
")



(provide :my_delta_robot/PositionArm "d2558a79329645ec1605ebea813b04a6")


