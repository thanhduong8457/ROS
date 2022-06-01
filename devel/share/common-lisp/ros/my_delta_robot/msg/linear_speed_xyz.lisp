; Auto-generated. Do not edit!


(cl:in-package my_delta_robot-msg)


;//! \htmlinclude linear_speed_xyz.msg.html

(cl:defclass <linear_speed_xyz> (roslisp-msg-protocol:ros-message)
  ((xo
    :reader xo
    :initarg :xo
    :type cl:float
    :initform 0.0)
   (yo
    :reader yo
    :initarg :yo
    :type cl:float
    :initform 0.0)
   (zo
    :reader zo
    :initarg :zo
    :type cl:float
    :initform 0.0)
   (xf
    :reader xf
    :initarg :xf
    :type cl:float
    :initform 0.0)
   (yf
    :reader yf
    :initarg :yf
    :type cl:float
    :initform 0.0)
   (zf
    :reader zf
    :initarg :zf
    :type cl:float
    :initform 0.0)
   (vmax
    :reader vmax
    :initarg :vmax
    :type cl:float
    :initform 0.0)
   (amax
    :reader amax
    :initarg :amax
    :type cl:float
    :initform 0.0)
   (gripper
    :reader gripper
    :initarg :gripper
    :type cl:integer
    :initform 0))
)

(cl:defclass linear_speed_xyz (<linear_speed_xyz>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <linear_speed_xyz>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'linear_speed_xyz)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_delta_robot-msg:<linear_speed_xyz> is deprecated: use my_delta_robot-msg:linear_speed_xyz instead.")))

(cl:ensure-generic-function 'xo-val :lambda-list '(m))
(cl:defmethod xo-val ((m <linear_speed_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:xo-val is deprecated.  Use my_delta_robot-msg:xo instead.")
  (xo m))

(cl:ensure-generic-function 'yo-val :lambda-list '(m))
(cl:defmethod yo-val ((m <linear_speed_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:yo-val is deprecated.  Use my_delta_robot-msg:yo instead.")
  (yo m))

(cl:ensure-generic-function 'zo-val :lambda-list '(m))
(cl:defmethod zo-val ((m <linear_speed_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:zo-val is deprecated.  Use my_delta_robot-msg:zo instead.")
  (zo m))

(cl:ensure-generic-function 'xf-val :lambda-list '(m))
(cl:defmethod xf-val ((m <linear_speed_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:xf-val is deprecated.  Use my_delta_robot-msg:xf instead.")
  (xf m))

(cl:ensure-generic-function 'yf-val :lambda-list '(m))
(cl:defmethod yf-val ((m <linear_speed_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:yf-val is deprecated.  Use my_delta_robot-msg:yf instead.")
  (yf m))

(cl:ensure-generic-function 'zf-val :lambda-list '(m))
(cl:defmethod zf-val ((m <linear_speed_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:zf-val is deprecated.  Use my_delta_robot-msg:zf instead.")
  (zf m))

(cl:ensure-generic-function 'vmax-val :lambda-list '(m))
(cl:defmethod vmax-val ((m <linear_speed_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:vmax-val is deprecated.  Use my_delta_robot-msg:vmax instead.")
  (vmax m))

(cl:ensure-generic-function 'amax-val :lambda-list '(m))
(cl:defmethod amax-val ((m <linear_speed_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:amax-val is deprecated.  Use my_delta_robot-msg:amax instead.")
  (amax m))

(cl:ensure-generic-function 'gripper-val :lambda-list '(m))
(cl:defmethod gripper-val ((m <linear_speed_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:gripper-val is deprecated.  Use my_delta_robot-msg:gripper instead.")
  (gripper m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <linear_speed_xyz>) ostream)
  "Serializes a message object of type '<linear_speed_xyz>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'xo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'xf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vmax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'amax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'gripper)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <linear_speed_xyz>) istream)
  "Deserializes a message object of type '<linear_speed_xyz>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xo) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yo) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zo) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xf) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yf) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zf) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vmax) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amax) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gripper) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<linear_speed_xyz>)))
  "Returns string type for a message object of type '<linear_speed_xyz>"
  "my_delta_robot/linear_speed_xyz")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'linear_speed_xyz)))
  "Returns string type for a message object of type 'linear_speed_xyz"
  "my_delta_robot/linear_speed_xyz")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<linear_speed_xyz>)))
  "Returns md5sum for a message object of type '<linear_speed_xyz>"
  "8043196b03be66e3bcd2ba1e7c560f64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'linear_speed_xyz)))
  "Returns md5sum for a message object of type 'linear_speed_xyz"
  "8043196b03be66e3bcd2ba1e7c560f64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<linear_speed_xyz>)))
  "Returns full string definition for message of type '<linear_speed_xyz>"
  (cl:format cl:nil "float32 xo~%float32 yo~%float32 zo~%float32 xf~%float32 yf~%float32 zf~%float32 vmax~%float32 amax~%int64 gripper~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'linear_speed_xyz)))
  "Returns full string definition for message of type 'linear_speed_xyz"
  (cl:format cl:nil "float32 xo~%float32 yo~%float32 zo~%float32 xf~%float32 yf~%float32 zf~%float32 vmax~%float32 amax~%int64 gripper~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <linear_speed_xyz>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <linear_speed_xyz>))
  "Converts a ROS message object to a list"
  (cl:list 'linear_speed_xyz
    (cl:cons ':xo (xo msg))
    (cl:cons ':yo (yo msg))
    (cl:cons ':zo (zo msg))
    (cl:cons ':xf (xf msg))
    (cl:cons ':yf (yf msg))
    (cl:cons ':zf (zf msg))
    (cl:cons ':vmax (vmax msg))
    (cl:cons ':amax (amax msg))
    (cl:cons ':gripper (gripper msg))
))
