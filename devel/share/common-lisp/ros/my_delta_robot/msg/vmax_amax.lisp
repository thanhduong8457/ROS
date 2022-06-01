; Auto-generated. Do not edit!


(cl:in-package my_delta_robot-msg)


;//! \htmlinclude vmax_amax.msg.html

(cl:defclass <vmax_amax> (roslisp-msg-protocol:ros-message)
  ((vmax
    :reader vmax
    :initarg :vmax
    :type cl:float
    :initform 0.0)
   (amax
    :reader amax
    :initarg :amax
    :type cl:float
    :initform 0.0))
)

(cl:defclass vmax_amax (<vmax_amax>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vmax_amax>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vmax_amax)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_delta_robot-msg:<vmax_amax> is deprecated: use my_delta_robot-msg:vmax_amax instead.")))

(cl:ensure-generic-function 'vmax-val :lambda-list '(m))
(cl:defmethod vmax-val ((m <vmax_amax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:vmax-val is deprecated.  Use my_delta_robot-msg:vmax instead.")
  (vmax m))

(cl:ensure-generic-function 'amax-val :lambda-list '(m))
(cl:defmethod amax-val ((m <vmax_amax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:amax-val is deprecated.  Use my_delta_robot-msg:amax instead.")
  (amax m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vmax_amax>) ostream)
  "Serializes a message object of type '<vmax_amax>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vmax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'amax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vmax_amax>) istream)
  "Deserializes a message object of type '<vmax_amax>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vmax) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amax) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vmax_amax>)))
  "Returns string type for a message object of type '<vmax_amax>"
  "my_delta_robot/vmax_amax")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vmax_amax)))
  "Returns string type for a message object of type 'vmax_amax"
  "my_delta_robot/vmax_amax")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vmax_amax>)))
  "Returns md5sum for a message object of type '<vmax_amax>"
  "2ab9faecee53d06a238561f5c5a17a83")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vmax_amax)))
  "Returns md5sum for a message object of type 'vmax_amax"
  "2ab9faecee53d06a238561f5c5a17a83")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vmax_amax>)))
  "Returns full string definition for message of type '<vmax_amax>"
  (cl:format cl:nil "float64 vmax~%float64 amax~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vmax_amax)))
  "Returns full string definition for message of type 'vmax_amax"
  (cl:format cl:nil "float64 vmax~%float64 amax~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vmax_amax>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vmax_amax>))
  "Converts a ROS message object to a list"
  (cl:list 'vmax_amax
    (cl:cons ':vmax (vmax msg))
    (cl:cons ':amax (amax msg))
))
