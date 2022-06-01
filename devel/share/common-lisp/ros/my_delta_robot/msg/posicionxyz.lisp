; Auto-generated. Do not edit!


(cl:in-package my_delta_robot-msg)


;//! \htmlinclude posicionxyz.msg.html

(cl:defclass <posicionxyz> (roslisp-msg-protocol:ros-message)
  ((x0
    :reader x0
    :initarg :x0
    :type cl:float
    :initform 0.0)
   (y0
    :reader y0
    :initarg :y0
    :type cl:float
    :initform 0.0)
   (z0
    :reader z0
    :initarg :z0
    :type cl:float
    :initform 0.0)
   (type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0))
)

(cl:defclass posicionxyz (<posicionxyz>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <posicionxyz>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'posicionxyz)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_delta_robot-msg:<posicionxyz> is deprecated: use my_delta_robot-msg:posicionxyz instead.")))

(cl:ensure-generic-function 'x0-val :lambda-list '(m))
(cl:defmethod x0-val ((m <posicionxyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:x0-val is deprecated.  Use my_delta_robot-msg:x0 instead.")
  (x0 m))

(cl:ensure-generic-function 'y0-val :lambda-list '(m))
(cl:defmethod y0-val ((m <posicionxyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:y0-val is deprecated.  Use my_delta_robot-msg:y0 instead.")
  (y0 m))

(cl:ensure-generic-function 'z0-val :lambda-list '(m))
(cl:defmethod z0-val ((m <posicionxyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:z0-val is deprecated.  Use my_delta_robot-msg:z0 instead.")
  (z0 m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <posicionxyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:type-val is deprecated.  Use my_delta_robot-msg:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <posicionxyz>) ostream)
  "Serializes a message object of type '<posicionxyz>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <posicionxyz>) istream)
  "Deserializes a message object of type '<posicionxyz>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<posicionxyz>)))
  "Returns string type for a message object of type '<posicionxyz>"
  "my_delta_robot/posicionxyz")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'posicionxyz)))
  "Returns string type for a message object of type 'posicionxyz"
  "my_delta_robot/posicionxyz")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<posicionxyz>)))
  "Returns md5sum for a message object of type '<posicionxyz>"
  "7254c4df121ba116f9b11125801d35f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'posicionxyz)))
  "Returns md5sum for a message object of type 'posicionxyz"
  "7254c4df121ba116f9b11125801d35f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<posicionxyz>)))
  "Returns full string definition for message of type '<posicionxyz>"
  (cl:format cl:nil "float32 x0~%float32 y0~%float32 z0~%int64 type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'posicionxyz)))
  "Returns full string definition for message of type 'posicionxyz"
  (cl:format cl:nil "float32 x0~%float32 y0~%float32 z0~%int64 type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <posicionxyz>))
  (cl:+ 0
     4
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <posicionxyz>))
  "Converts a ROS message object to a list"
  (cl:list 'posicionxyz
    (cl:cons ':x0 (x0 msg))
    (cl:cons ':y0 (y0 msg))
    (cl:cons ':z0 (z0 msg))
    (cl:cons ':type (type msg))
))
