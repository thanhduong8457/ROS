; Auto-generated. Do not edit!


(cl:in-package my_delta_robot-msg)


;//! \htmlinclude PositionArm.msg.html

(cl:defclass <PositionArm> (roslisp-msg-protocol:ros-message)
  ((base_brazo1
    :reader base_brazo1
    :initarg :base_brazo1
    :type cl:float
    :initform 0.0)
   (base_brazo2
    :reader base_brazo2
    :initarg :base_brazo2
    :type cl:float
    :initform 0.0)
   (base_brazo3
    :reader base_brazo3
    :initarg :base_brazo3
    :type cl:float
    :initform 0.0)
   (codo1_a
    :reader codo1_a
    :initarg :codo1_a
    :type cl:float
    :initform 0.0)
   (codo1_b
    :reader codo1_b
    :initarg :codo1_b
    :type cl:float
    :initform 0.0)
   (codo2_a
    :reader codo2_a
    :initarg :codo2_a
    :type cl:float
    :initform 0.0)
   (codo2_b
    :reader codo2_b
    :initarg :codo2_b
    :type cl:float
    :initform 0.0)
   (codo3_a
    :reader codo3_a
    :initarg :codo3_a
    :type cl:float
    :initform 0.0)
   (codo3_b
    :reader codo3_b
    :initarg :codo3_b
    :type cl:float
    :initform 0.0)
   (act_x
    :reader act_x
    :initarg :act_x
    :type cl:float
    :initform 0.0)
   (act_y
    :reader act_y
    :initarg :act_y
    :type cl:float
    :initform 0.0)
   (act_z
    :reader act_z
    :initarg :act_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass PositionArm (<PositionArm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionArm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionArm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_delta_robot-msg:<PositionArm> is deprecated: use my_delta_robot-msg:PositionArm instead.")))

(cl:ensure-generic-function 'base_brazo1-val :lambda-list '(m))
(cl:defmethod base_brazo1-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:base_brazo1-val is deprecated.  Use my_delta_robot-msg:base_brazo1 instead.")
  (base_brazo1 m))

(cl:ensure-generic-function 'base_brazo2-val :lambda-list '(m))
(cl:defmethod base_brazo2-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:base_brazo2-val is deprecated.  Use my_delta_robot-msg:base_brazo2 instead.")
  (base_brazo2 m))

(cl:ensure-generic-function 'base_brazo3-val :lambda-list '(m))
(cl:defmethod base_brazo3-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:base_brazo3-val is deprecated.  Use my_delta_robot-msg:base_brazo3 instead.")
  (base_brazo3 m))

(cl:ensure-generic-function 'codo1_a-val :lambda-list '(m))
(cl:defmethod codo1_a-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:codo1_a-val is deprecated.  Use my_delta_robot-msg:codo1_a instead.")
  (codo1_a m))

(cl:ensure-generic-function 'codo1_b-val :lambda-list '(m))
(cl:defmethod codo1_b-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:codo1_b-val is deprecated.  Use my_delta_robot-msg:codo1_b instead.")
  (codo1_b m))

(cl:ensure-generic-function 'codo2_a-val :lambda-list '(m))
(cl:defmethod codo2_a-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:codo2_a-val is deprecated.  Use my_delta_robot-msg:codo2_a instead.")
  (codo2_a m))

(cl:ensure-generic-function 'codo2_b-val :lambda-list '(m))
(cl:defmethod codo2_b-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:codo2_b-val is deprecated.  Use my_delta_robot-msg:codo2_b instead.")
  (codo2_b m))

(cl:ensure-generic-function 'codo3_a-val :lambda-list '(m))
(cl:defmethod codo3_a-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:codo3_a-val is deprecated.  Use my_delta_robot-msg:codo3_a instead.")
  (codo3_a m))

(cl:ensure-generic-function 'codo3_b-val :lambda-list '(m))
(cl:defmethod codo3_b-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:codo3_b-val is deprecated.  Use my_delta_robot-msg:codo3_b instead.")
  (codo3_b m))

(cl:ensure-generic-function 'act_x-val :lambda-list '(m))
(cl:defmethod act_x-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:act_x-val is deprecated.  Use my_delta_robot-msg:act_x instead.")
  (act_x m))

(cl:ensure-generic-function 'act_y-val :lambda-list '(m))
(cl:defmethod act_y-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:act_y-val is deprecated.  Use my_delta_robot-msg:act_y instead.")
  (act_y m))

(cl:ensure-generic-function 'act_z-val :lambda-list '(m))
(cl:defmethod act_z-val ((m <PositionArm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_delta_robot-msg:act_z-val is deprecated.  Use my_delta_robot-msg:act_z instead.")
  (act_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionArm>) ostream)
  "Serializes a message object of type '<PositionArm>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'base_brazo1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'base_brazo2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'base_brazo3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'codo1_a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'codo1_b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'codo2_a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'codo2_b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'codo3_a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'codo3_b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'act_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'act_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'act_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionArm>) istream)
  "Deserializes a message object of type '<PositionArm>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'base_brazo1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'base_brazo2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'base_brazo3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'codo1_a) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'codo1_b) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'codo2_a) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'codo2_b) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'codo3_a) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'codo3_b) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'act_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'act_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'act_z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionArm>)))
  "Returns string type for a message object of type '<PositionArm>"
  "my_delta_robot/PositionArm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionArm)))
  "Returns string type for a message object of type 'PositionArm"
  "my_delta_robot/PositionArm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionArm>)))
  "Returns md5sum for a message object of type '<PositionArm>"
  "d2558a79329645ec1605ebea813b04a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionArm)))
  "Returns md5sum for a message object of type 'PositionArm"
  "d2558a79329645ec1605ebea813b04a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionArm>)))
  "Returns full string definition for message of type '<PositionArm>"
  (cl:format cl:nil "float64 base_brazo1~%float64 base_brazo2~%float64 base_brazo3~%float64 codo1_a~%float64 codo1_b~%float64 codo2_a~%float64 codo2_b~%float64 codo3_a~%float64 codo3_b~%float64 act_x~%float64 act_y~%float64 act_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionArm)))
  "Returns full string definition for message of type 'PositionArm"
  (cl:format cl:nil "float64 base_brazo1~%float64 base_brazo2~%float64 base_brazo3~%float64 codo1_a~%float64 codo1_b~%float64 codo2_a~%float64 codo2_b~%float64 codo3_a~%float64 codo3_b~%float64 act_x~%float64 act_y~%float64 act_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionArm>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionArm>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionArm
    (cl:cons ':base_brazo1 (base_brazo1 msg))
    (cl:cons ':base_brazo2 (base_brazo2 msg))
    (cl:cons ':base_brazo3 (base_brazo3 msg))
    (cl:cons ':codo1_a (codo1_a msg))
    (cl:cons ':codo1_b (codo1_b msg))
    (cl:cons ':codo2_a (codo2_a msg))
    (cl:cons ':codo2_b (codo2_b msg))
    (cl:cons ':codo3_a (codo3_a msg))
    (cl:cons ':codo3_b (codo3_b msg))
    (cl:cons ':act_x (act_x msg))
    (cl:cons ':act_y (act_y msg))
    (cl:cons ':act_z (act_z msg))
))
