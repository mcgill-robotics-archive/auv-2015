; Auto-generated. Do not edit!


(cl:in-package auv_msgs-msg)


;//! \htmlinclude CVTarget.msg.html

(cl:defclass <CVTarget> (roslisp-msg-protocol:ros-message)
  ((CVTarget
    :reader CVTarget
    :initarg :CVTarget
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CVTarget (<CVTarget>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CVTarget>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CVTarget)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name auv_msgs-msg:<CVTarget> is deprecated: use auv_msgs-msg:CVTarget instead.")))

(cl:ensure-generic-function 'CVTarget-val :lambda-list '(m))
(cl:defmethod CVTarget-val ((m <CVTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:CVTarget-val is deprecated.  Use auv_msgs-msg:CVTarget instead.")
  (CVTarget m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CVTarget>)))
    "Constants for message type '<CVTarget>"
  '((:NOTHING . 0)
    (:GATE . 1)
    (:BUOY . 2)
    (:LANE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CVTarget)))
    "Constants for message type 'CVTarget"
  '((:NOTHING . 0)
    (:GATE . 1)
    (:BUOY . 2)
    (:LANE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CVTarget>) ostream)
  "Serializes a message object of type '<CVTarget>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CVTarget)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CVTarget>) istream)
  "Deserializes a message object of type '<CVTarget>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CVTarget)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CVTarget>)))
  "Returns string type for a message object of type '<CVTarget>"
  "auv_msgs/CVTarget")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CVTarget)))
  "Returns string type for a message object of type 'CVTarget"
  "auv_msgs/CVTarget")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CVTarget>)))
  "Returns md5sum for a message object of type '<CVTarget>"
  "1bcd46034f8c9e5490893c5a24fa63fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CVTarget)))
  "Returns md5sum for a message object of type 'CVTarget"
  "1bcd46034f8c9e5490893c5a24fa63fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CVTarget>)))
  "Returns full string definition for message of type '<CVTarget>"
  (cl:format cl:nil "uint8 CVTarget~%~%uint8 NOTHING=0~%uint8 GATE=1~%uint8 BUOY=2~%uint8 LANE=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CVTarget)))
  "Returns full string definition for message of type 'CVTarget"
  (cl:format cl:nil "uint8 CVTarget~%~%uint8 NOTHING=0~%uint8 GATE=1~%uint8 BUOY=2~%uint8 LANE=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CVTarget>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CVTarget>))
  "Converts a ROS message object to a list"
  (cl:list 'CVTarget
    (cl:cons ':CVTarget (CVTarget msg))
))
