;; Auto-generated. Do not edit!


(when (boundp 'my_delta_robot::image_pos)
  (if (not (find-package "MY_DELTA_ROBOT"))
    (make-package "MY_DELTA_ROBOT"))
  (shadow 'image_pos (find-package "MY_DELTA_ROBOT")))
(unless (find-package "MY_DELTA_ROBOT::IMAGE_POS")
  (make-package "MY_DELTA_ROBOT::IMAGE_POS"))

(in-package "ROS")
;;//! \htmlinclude image_pos.msg.html


(defclass my_delta_robot::image_pos
  :super ros::object
  :slots (_x _y _z _type ))

(defmethod my_delta_robot::image_pos
  (:init
   (&key
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:z __z) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:type __type) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   (setq _z __z)
   (setq _type __type)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:serialization-length
   ()
   (+
    ;; float32[] _x
    (* 4    (length _x)) 4
    ;; float32[] _y
    (* 4    (length _y)) 4
    ;; float32[] _z
    (* 4    (length _z)) 4
    ;; int64[] _type
    (* 8    (length _type)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _z
     (write-long (length _z) s)
     (dotimes (i (length _z))
       (sys::poke (elt _z i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int64[] _type
     (write-long (length _type) s)
     (dotimes (i (length _type))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _type i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _type i)) (= (length ((elt _type i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _type i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _type i) . bv) 1) -1) s))
             ((and (class (elt _type i)) (= (length ((elt _type i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _type i) . bv) 0) s)
              (write-long (if (>= (elt _type i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _type i) s)(write-long (if (>= (elt _type i) 0) 0 #xffffffff) s)))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _z i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int64[] _type
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _type (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _type i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _type i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;;
   self)
  )

(setf (get my_delta_robot::image_pos :md5sum-) "6e41facbac095be2059fd056239eeb7d")
(setf (get my_delta_robot::image_pos :datatype-) "my_delta_robot/image_pos")
(setf (get my_delta_robot::image_pos :definition-)
      "float32[] x
float32[] y
float32[] z
int64[] type
")



(provide :my_delta_robot/image_pos "6e41facbac095be2059fd056239eeb7d")


