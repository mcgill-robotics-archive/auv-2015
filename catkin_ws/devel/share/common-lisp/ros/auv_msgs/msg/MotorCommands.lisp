; Auto-generated. Do not edit!


(cl:in-package auv_msgs-msg)


;//! \htmlinclude MotorCommands.msg.html

(cl:defclass <MotorCommands> (roslisp-msg-protocol:ros-message)
  ((cmd_surge_starboard
    :reader cmd_surge_starboard
    :initarg :cmd_surge_starboard
    :type cl:integer
    :initform 0)
   (cmd_surge_port
    :reader cmd_surge_port
    :initarg :cmd_surge_port
    :type cl:integer
    :initform 0)
   (cmd_sway_bow
    :reader cmd_sway_bow
    :initarg :cmd_sway_bow
    :type cl:integer
    :initform 0)
   (cmd_sway_stern
    :reader cmd_sway_stern
    :initarg :cmd_sway_stern
    :type cl:integer
    :initform 0)
   (cmd_heave_bow
    :reader cmd_heave_bow
    :initarg :cmd_heave_bow
    :type cl:integer
    :initform 0)
   (cmd_heave_stern
    :reader cmd_heave_stern
    :initarg :cmd_heave_stern
    :type cl:integer
    :initform 0))
)

(cl:defclass MotorCommands (<MotorCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name auv_msgs-msg:<MotorCommands> is deprecated: use auv_msgs-msg:MotorCommands instead.")))

(cl:ensure-generic-function 'cmd_surge_starboard-val :lambda-list '(m))
(cl:defmethod cmd_surge_starboard-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:cmd_surge_starboard-val is deprecated.  Use auv_msgs-msg:cmd_surge_starboard instead.")
  (cmd_surge_starboard m))

(cl:ensure-generic-function 'cmd_surge_port-val :lambda-list '(m))
(cl:defmethod cmd_surge_port-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:cmd_surge_port-val is deprecated.  Use auv_msgs-msg:cmd_surge_port instead.")
  (cmd_surge_port m))

(cl:ensure-generic-function 'cmd_sway_bow-val :lambda-list '(m))
(cl:defmethod cmd_sway_bow-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:cmd_sway_bow-val is deprecated.  Use auv_msgs-msg:cmd_sway_bow instead.")
  (cmd_sway_bow m))

(cl:ensure-generic-function 'cmd_sway_stern-val :lambda-list '(m))
(cl:defmethod cmd_sway_stern-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:cmd_sway_stern-val is deprecated.  Use auv_msgs-msg:cmd_sway_stern instead.")
  (cmd_sway_stern m))

(cl:ensure-generic-function 'cmd_heave_bow-val :lambda-list '(m))
(cl:defmethod cmd_heave_bow-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:cmd_heave_bow-val is deprecated.  Use auv_msgs-msg:cmd_heave_bow instead.")
  (cmd_heave_bow m))

(cl:ensure-generic-function 'cmd_heave_stern-val :lambda-list '(m))
(cl:defmethod cmd_heave_stern-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:cmd_heave_stern-val is deprecated.  Use auv_msgs-msg:cmd_heave_stern instead.")
  (cmd_heave_stern m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorCommands>) ostream)
  "Serializes a message object of type '<MotorCommands>"
  (cl:let* ((signed (cl:slot-value msg 'cmd_surge_starboard)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cmd_surge_port)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cmd_sway_bow)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cmd_sway_stern)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cmd_heave_bow)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cmd_heave_stern)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorCommands>) istream)
  "Deserializes a message object of type '<MotorCommands>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd_surge_starboard) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd_surge_port) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd_sway_bow) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd_sway_stern) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd_heave_bow) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd_heave_stern) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorCommands>)))
  "Returns string type for a message object of type '<MotorCommands>"
  "auv_msgs/MotorCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorCommands)))
  "Returns string type for a message object of type 'MotorCommands"
  "auv_msgs/MotorCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorCommands>)))
  "Returns md5sum for a message object of type '<MotorCommands>"
  "ea2d5f40a47b1880926c752b1d9424bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorCommands)))
  "Returns md5sum for a message object of type 'MotorCommands"
  "ea2d5f40a47b1880926c752b1d9424bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorCommands>)))
  "Returns full string definition for message of type '<MotorCommands>"
  (cl:format cl:nil "int32 cmd_surge_starboard~%int32 cmd_surge_port~%int32 cmd_sway_bow~%int32 cmd_sway_stern~%int32 cmd_heave_bow~%int32 cmd_heave_stern~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorCommands)))
  "Returns full string definition for message of type 'MotorCommands"
  (cl:format cl:nil "int32 cmd_surge_starboard~%int32 cmd_surge_port~%int32 cmd_sway_bow~%int32 cmd_sway_stern~%int32 cmd_heave_bow~%int32 cmd_heave_stern~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorCommands>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorCommands
    (cl:cons ':cmd_surge_starboard (cmd_surge_starboard msg))
    (cl:cons ':cmd_surge_port (cmd_surge_port msg))
    (cl:cons ':cmd_sway_bow (cmd_sway_bow msg))
    (cl:cons ':cmd_sway_stern (cmd_sway_stern msg))
    (cl:cons ':cmd_heave_bow (cmd_heave_bow msg))
    (cl:cons ':cmd_heave_stern (cmd_heave_stern msg))
))
