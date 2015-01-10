; Auto-generated. Do not edit!


(cl:in-package auv_msgs-msg)


;//! \htmlinclude SonarTarget.msg.html

(cl:defclass <SonarTarget> (roslisp-msg-protocol:ros-message)
  ((SonarTarget
    :reader SonarTarget
    :initarg :SonarTarget
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SonarTarget (<SonarTarget>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SonarTarget>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SonarTarget)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name auv_msgs-msg:<SonarTarget> is deprecated: use auv_msgs-msg:SonarTarget instead.")))

(cl:ensure-generic-function 'SonarTarget-val :lambda-list '(m))
(cl:defmethod SonarTarget-val ((m <SonarTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auv_msgs-msg:SonarTarget-val is deprecated.  Use auv_msgs-msg:SonarTarget instead.")
  (SonarTarget m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SonarTarget>)))
    "Constants for message type '<SonarTarget>"
  '((:NOTHING . 0)
    (:GATE . 1)
    (:BUOY . 2)
    (:LANE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SonarTarget)))
    "Constants for message type 'SonarTarget"
  '((:NOTHING . 0)
    (:GATE . 1)
    (:BUOY . 2)
    (:LANE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SonarTarget>) ostream)
  "Serializes a message object of type '<SonarTarget>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SonarTarget)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SonarTarget>) istream)
  "Deserializes a message object of type '<SonarTarget>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SonarTarget)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SonarTarget>)))
  "Returns string type for a message object of type '<SonarTarget>"
  "auv_msgs/SonarTarget")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SonarTarget)))
  "Returns string type for a message object of type 'SonarTarget"
  "auv_msgs/SonarTarget")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SonarTarget>)))
  "Returns md5sum for a message object of type '<SonarTarget>"
  "f9a17e18cc0865911545e5034f45cd00")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SonarTarget)))
  "Returns md5sum for a message object of type 'SonarTarget"
  "f9a17e18cc0865911545e5034f45cd00")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SonarTarget>)))
  "Returns full string definition for message of type '<SonarTarget>"
  (cl:format cl:nil "uint8 SonarTarget~%~%#just C/Ped from CVTarget, will update when we get there~%uint8 NOTHING=0~%uint8 GATE=1~%uint8 BUOY=2~%uint8 LANE=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SonarTarget)))
  "Returns full string definition for message of type 'SonarTarget"
  (cl:format cl:nil "uint8 SonarTarget~%~%#just C/Ped from CVTarget, will update when we get there~%uint8 NOTHING=0~%uint8 GATE=1~%uint8 BUOY=2~%uint8 LANE=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SonarTarget>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SonarTarget>))
  "Converts a ROS message object to a list"
  (cl:list 'SonarTarget
    (cl:cons ':SonarTarget (SonarTarget msg))
))
