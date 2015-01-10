; Auto-generated. Do not edit!


(cl:in-package auv_msgs-msg)


;//! \htmlinclude Solenoid.msg.html

(cl:defclass <Solenoid> (roslisp-msg-protocol:ros-message)
  ((solenoid1
    :reader solenoid1
    :initarg :solenoid1
    :type cl:boolean
    :initform cl:nil)
   (solenoid2
    :reader solenoid2
    :initarg :solenoid2
    :type cl:boolean
    :initform cl:nil)
   (solenoid3
    :reader solenoid3
    :initarg :solenoid3
    :type cl:boolean
    :initform cl:nil)
   (solenoid4
    :reader solenoid4
    :initarg :solenoid4
    :type cl:boolean
    :initform cl:nil)
   (solenoid5
    :reader solenoid5
    :initarg :solenoid5
    :type cl:boolean
    :initform cl:nil)
   (solenoid6
    :reader solenoid6
    :initarg :solenoid6
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Solenoid (<Solenoid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Solenoid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Solenoid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name auv_msgs-msg:<Solenoid> is deprecated: use auv_msgs-msg:Solenoid instead.")))

(cl:ensure-generic-function 'solenoid1-val :lambda-list '(m))
(cl:defmethod solenoid1-val ((m <Solenoid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:solenoid1-val is deprecated.  Use auv_msgs-msg:solenoid1 instead.")
  (solenoid1 m))

(cl:ensure-generic-function 'solenoid2-val :lambda-list '(m))
(cl:defmethod solenoid2-val ((m <Solenoid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:solenoid2-val is deprecated.  Use auv_msgs-msg:solenoid2 instead.")
  (solenoid2 m))

(cl:ensure-generic-function 'solenoid3-val :lambda-list '(m))
(cl:defmethod solenoid3-val ((m <Solenoid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:solenoid3-val is deprecated.  Use auv_msgs-msg:solenoid3 instead.")
  (solenoid3 m))

(cl:ensure-generic-function 'solenoid4-val :lambda-list '(m))
(cl:defmethod solenoid4-val ((m <Solenoid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:solenoid4-val is deprecated.  Use auv_msgs-msg:solenoid4 instead.")
  (solenoid4 m))

(cl:ensure-generic-function 'solenoid5-val :lambda-list '(m))
(cl:defmethod solenoid5-val ((m <Solenoid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:solenoid5-val is deprecated.  Use auv_msgs-msg:solenoid5 instead.")
  (solenoid5 m))

(cl:ensure-generic-function 'solenoid6-val :lambda-list '(m))
(cl:defmethod solenoid6-val ((m <Solenoid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:solenoid6-val is deprecated.  Use auv_msgs-msg:solenoid6 instead.")
  (solenoid6 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Solenoid>) ostream)
  "Serializes a message object of type '<Solenoid>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'solenoid1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'solenoid2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'solenoid3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'solenoid4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'solenoid5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'solenoid6) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Solenoid>) istream)
  "Deserializes a message object of type '<Solenoid>"
    (cl:setf (cl:slot-value msg 'solenoid1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'solenoid2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'solenoid3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'solenoid4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'solenoid5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'solenoid6) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Solenoid>)))
  "Returns string type for a message object of type '<Solenoid>"
  "auv_msgs/Solenoid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Solenoid)))
  "Returns string type for a message object of type 'Solenoid"
  "auv_msgs/Solenoid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Solenoid>)))
  "Returns md5sum for a message object of type '<Solenoid>"
  "0efcb9185f05bca53eb2775d2b56d3f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Solenoid)))
  "Returns md5sum for a message object of type 'Solenoid"
  "0efcb9185f05bca53eb2775d2b56d3f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Solenoid>)))
  "Returns full string definition for message of type '<Solenoid>"
  (cl:format cl:nil "bool solenoid1~%bool solenoid2~%bool solenoid3~%bool solenoid4~%bool solenoid5~%bool solenoid6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Solenoid)))
  "Returns full string definition for message of type 'Solenoid"
  (cl:format cl:nil "bool solenoid1~%bool solenoid2~%bool solenoid3~%bool solenoid4~%bool solenoid5~%bool solenoid6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Solenoid>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Solenoid>))
  "Converts a ROS message object to a list"
  (cl:list 'Solenoid
    (cl:cons ':solenoid1 (solenoid1 msg))
    (cl:cons ':solenoid2 (solenoid2 msg))
    (cl:cons ':solenoid3 (solenoid3 msg))
    (cl:cons ':solenoid4 (solenoid4 msg))
    (cl:cons ':solenoid5 (solenoid5 msg))
    (cl:cons ':solenoid6 (solenoid6 msg))
))
