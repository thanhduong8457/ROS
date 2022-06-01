;; Auto-generated. Do not edit!


(when (boundp 'my_delta_robot::num_point)
  (if (not (find-package "MY_DELTA_ROBOT"))
    (make-package "MY_DELTA_ROBOT"))
  (shadow 'num_point (find-package "MY_DELTA_ROBOT")))
(unless (find-package "MY_DELTA_ROBOT::NUM_POINT")
  (make-package "MY_DELTA_ROBOT::NUM_POINT"))

(in-package "ROS")
;;//! \htmlinclude num_point.msg.html


(defclass my_delta_robot::num_point
  :super ros::object
  :slots (_num_point_1 _num_point_2 ))

(defmethod my_delta_robot::num_point
  (:init
   (&key
    ((:num_point_1 __num_point_1) 0)
    ((:num_point_2 __num_point_2) 0)
    )
   (send-super :init)
   (setq _num_point_1 (round __num_point_1))
   (setq _num_point_2 (round __num_point_2))
   self)
  (:num_point_1
   (&optional __num_point_1)
   (if __num_point_1 (setq _num_point_1 __num_point_1)) _num_point_1)
  (:num_point_2
   (&optional __num_point_2)
   (if __num_point_2 (setq _num_point_2 __num_point_2)) _num_point_2)
  (:serialization-length
   ()
   (+
    ;; int64 _num_point_1
    8
    ;; int64 _num_point_2
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _num_point_1
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_point_1 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_point_1) (= (length (_num_point_1 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_point_1 . bv) 0) 0) s)
              (write-long (ash (elt (_num_point_1 . bv) 1) -1) s))
             ((and (class _num_point_1) (= (length (_num_point_1 . bv)) 1)) ;; big1
              (write-long (elt (_num_point_1 . bv) 0) s)
              (write-long (if (>= _num_point_1 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_point_1 s)(write-long (if (>= _num_point_1 0) 0 #xffffffff) s)))
     ;; int64 _num_point_2
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_point_2 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_point_2) (= (length (_num_point_2 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_point_2 . bv) 0) 0) s)
              (write-long (ash (elt (_num_point_2 . bv) 1) -1) s))
             ((and (class _num_point_2) (= (length (_num_point_2 . bv)) 1)) ;; big1
              (write-long (elt (_num_point_2 . bv) 0) s)
              (write-long (if (>= _num_point_2 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_point_2 s)(write-long (if (>= _num_point_2 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _num_point_1
#+(or :alpha :irix6 :x86_64)
      (setf _num_point_1 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_point_1 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _num_point_2
#+(or :alpha :irix6 :x86_64)
      (setf _num_point_2 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_point_2 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(setf (get my_delta_robot::num_point :md5sum-) "f69ff62ad97702372d7a82c71cf0ac27")
(setf (get my_delta_robot::num_point :datatype-) "my_delta_robot/num_point")
(setf (get my_delta_robot::num_point :definition-)
      "int64 num_point_1
int64 num_point_2
")



(provide :my_delta_robot/num_point "f69ff62ad97702372d7a82c71cf0ac27")


