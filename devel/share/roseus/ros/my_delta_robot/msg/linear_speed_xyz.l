;; Auto-generated. Do not edit!


(when (boundp 'my_delta_robot::linear_speed_xyz)
  (if (not (find-package "MY_DELTA_ROBOT"))
    (make-package "MY_DELTA_ROBOT"))
  (shadow 'linear_speed_xyz (find-package "MY_DELTA_ROBOT")))
(unless (find-package "MY_DELTA_ROBOT::LINEAR_SPEED_XYZ")
  (make-package "MY_DELTA_ROBOT::LINEAR_SPEED_XYZ"))

(in-package "ROS")
;;//! \htmlinclude linear_speed_xyz.msg.html


(defclass my_delta_robot::linear_speed_xyz
  :super ros::object
  :slots (_xo _yo _zo _xf _yf _zf _vmax _amax _gripper ))

(defmethod my_delta_robot::linear_speed_xyz
  (:init
   (&key
    ((:xo __xo) 0.0)
    ((:yo __yo) 0.0)
    ((:zo __zo) 0.0)
    ((:xf __xf) 0.0)
    ((:yf __yf) 0.0)
    ((:zf __zf) 0.0)
    ((:vmax __vmax) 0.0)
    ((:amax __amax) 0.0)
    ((:gripper __gripper) 0)
    )
   (send-super :init)
   (setq _xo (float __xo))
   (setq _yo (float __yo))
   (setq _zo (float __zo))
   (setq _xf (float __xf))
   (setq _yf (float __yf))
   (setq _zf (float __zf))
   (setq _vmax (float __vmax))
   (setq _amax (float __amax))
   (setq _gripper (round __gripper))
   self)
  (:xo
   (&optional __xo)
   (if __xo (setq _xo __xo)) _xo)
  (:yo
   (&optional __yo)
   (if __yo (setq _yo __yo)) _yo)
  (:zo
   (&optional __zo)
   (if __zo (setq _zo __zo)) _zo)
  (:xf
   (&optional __xf)
   (if __xf (setq _xf __xf)) _xf)
  (:yf
   (&optional __yf)
   (if __yf (setq _yf __yf)) _yf)
  (:zf
   (&optional __zf)
   (if __zf (setq _zf __zf)) _zf)
  (:vmax
   (&optional __vmax)
   (if __vmax (setq _vmax __vmax)) _vmax)
  (:amax
   (&optional __amax)
   (if __amax (setq _amax __amax)) _amax)
  (:gripper
   (&optional __gripper)
   (if __gripper (setq _gripper __gripper)) _gripper)
  (:serialization-length
   ()
   (+
    ;; float32 _xo
    4
    ;; float32 _yo
    4
    ;; float32 _zo
    4
    ;; float32 _xf
    4
    ;; float32 _yf
    4
    ;; float32 _zf
    4
    ;; float32 _vmax
    4
    ;; float32 _amax
    4
    ;; int64 _gripper
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _xo
       (sys::poke _xo (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yo
       (sys::poke _yo (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _zo
       (sys::poke _zo (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _xf
       (sys::poke _xf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yf
       (sys::poke _yf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _zf
       (sys::poke _zf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vmax
       (sys::poke _vmax (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _amax
       (sys::poke _amax (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int64 _gripper
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _gripper (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _gripper) (= (length (_gripper . bv)) 2)) ;; bignum
              (write-long (ash (elt (_gripper . bv) 0) 0) s)
              (write-long (ash (elt (_gripper . bv) 1) -1) s))
             ((and (class _gripper) (= (length (_gripper . bv)) 1)) ;; big1
              (write-long (elt (_gripper . bv) 0) s)
              (write-long (if (>= _gripper 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _gripper s)(write-long (if (>= _gripper 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _xo
     (setq _xo (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yo
     (setq _yo (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _zo
     (setq _zo (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _xf
     (setq _xf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yf
     (setq _yf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _zf
     (setq _zf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vmax
     (setq _vmax (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _amax
     (setq _amax (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int64 _gripper
#+(or :alpha :irix6 :x86_64)
      (setf _gripper (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _gripper (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(setf (get my_delta_robot::linear_speed_xyz :md5sum-) "8043196b03be66e3bcd2ba1e7c560f64")
(setf (get my_delta_robot::linear_speed_xyz :datatype-) "my_delta_robot/linear_speed_xyz")
(setf (get my_delta_robot::linear_speed_xyz :definition-)
      "float32 xo
float32 yo
float32 zo
float32 xf
float32 yf
float32 zf
float32 vmax
float32 amax
int64 gripper
")



(provide :my_delta_robot/linear_speed_xyz "8043196b03be66e3bcd2ba1e7c560f64")


