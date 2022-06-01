;; Auto-generated. Do not edit!


(when (boundp 'my_delta_robot::posicionxyz)
  (if (not (find-package "MY_DELTA_ROBOT"))
    (make-package "MY_DELTA_ROBOT"))
  (shadow 'posicionxyz (find-package "MY_DELTA_ROBOT")))
(unless (find-package "MY_DELTA_ROBOT::POSICIONXYZ")
  (make-package "MY_DELTA_ROBOT::POSICIONXYZ"))

(in-package "ROS")
;;//! \htmlinclude posicionxyz.msg.html


(defclass my_delta_robot::posicionxyz
  :super ros::object
  :slots (_x0 _y0 _z0 _type ))

(defmethod my_delta_robot::posicionxyz
  (:init
   (&key
    ((:x0 __x0) 0.0)
    ((:y0 __y0) 0.0)
    ((:z0 __z0) 0.0)
    ((:type __type) 0)
    )
   (send-super :init)
   (setq _x0 (float __x0))
   (setq _y0 (float __y0))
   (setq _z0 (float __z0))
   (setq _type (round __type))
   self)
  (:x0
   (&optional __x0)
   (if __x0 (setq _x0 __x0)) _x0)
  (:y0
   (&optional __y0)
   (if __y0 (setq _y0 __y0)) _y0)
  (:z0
   (&optional __z0)
   (if __z0 (setq _z0 __z0)) _z0)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:serialization-length
   ()
   (+
    ;; float32 _x0
    4
    ;; float32 _y0
    4
    ;; float32 _z0
    4
    ;; int64 _type
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x0
       (sys::poke _x0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y0
       (sys::poke _y0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z0
       (sys::poke _z0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int64 _type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _type) (= (length (_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_type . bv) 0) 0) s)
              (write-long (ash (elt (_type . bv) 1) -1) s))
             ((and (class _type) (= (length (_type . bv)) 1)) ;; big1
              (write-long (elt (_type . bv) 0) s)
              (write-long (if (>= _type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _type s)(write-long (if (>= _type 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x0
     (setq _x0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y0
     (setq _y0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z0
     (setq _z0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int64 _type
#+(or :alpha :irix6 :x86_64)
      (setf _type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get my_delta_robot::posicionxyz :md5sum-) "7254c4df121ba116f9b11125801d35f6")
(setf (get my_delta_robot::posicionxyz :datatype-) "my_delta_robot/posicionxyz")
(setf (get my_delta_robot::posicionxyz :definition-)
      "float32 x0
float32 y0
float32 z0
int64 type
")



(provide :my_delta_robot/posicionxyz "7254c4df121ba116f9b11125801d35f6")


