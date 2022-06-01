; Auto-generated. Do not edit!


(cl:in-package my_delta_robot-msg)


;//! \htmlinclude num_point.msg.html

(cl:defclass <num_point> (roslisp-msg-protocol:ros-message)
  ((num_point_1
    :reader num_point_1
    :initarg :num_point_1
    :type cl:integer
    :initform 0)
   (num_point_2
    :reader num_point_2
    :initarg :num_point_2
    :type cl:integer
    :initform 0))
)

(cl:defclass num_point (<num_point>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <num_point>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'num_point)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_delta_robot-msg:<num_point> is deprecated: use my_delta_robot-msg:num_point instead.")))

(cl:ensure-generic-function 'num_point_1-val :lambda-list '(m))
(cl:defmethod num_point_1-val ((m <num_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:num_point_1-val is deprecated.  Use my_delta_robot-msg:num_point_1 instead.")
  (num_point_1 m))

(cl:ensure-generic-function 'num_point_2-val :lambda-list '(m))
(cl:defmethod num_point_2-val ((m <num_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:num_point_2-val is deprecated.  Use my_delta_robot-msg:num_point_2 instead.")
  (num_point_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <num_point>) ostream)
  "Serializes a message object of type '<num_point>"
  (cl:let* ((signed (cl:slot-value msg 'num_point_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_point_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <num_point>) istream)
  "Deserializes a message object of type '<num_point>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_point_1) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_point_2) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<num_point>)))
  "Returns string type for a message object of type '<num_point>"
  "my_delta_robot/num_point")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'num_point)))
  "Returns string type for a message object of type 'num_point"
  "my_delta_robot/num_point")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<num_point>)))
  "Returns md5sum for a message object of type '<num_point>"
  "f69ff62ad97702372d7a82c71cf0ac27")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'num_point)))
  "Returns md5sum for a message object of type 'num_point"
  "f69ff62ad97702372d7a82c71cf0ac27")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<num_point>)))
  "Returns full string definition for message of type '<num_point>"
  (cl:format cl:nil "int64 num_point_1~%int64 num_point_2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'num_point)))
  "Returns full string definition for message of type 'num_point"
  (cl:format cl:nil "int64 num_point_1~%int64 num_point_2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <num_point>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <num_point>))
  "Converts a ROS message object to a list"
  (cl:list 'num_point
    (cl:cons ':num_point_1 (num_point_1 msg))
    (cl:cons ':num_point_2 (num_point_2 msg))
))
