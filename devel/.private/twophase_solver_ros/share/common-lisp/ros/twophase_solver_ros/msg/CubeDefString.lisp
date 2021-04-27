; Auto-generated. Do not edit!


(cl:in-package twophase_solver_ros-msg)


;//! \htmlinclude CubeDefString.msg.html

(cl:defclass <CubeDefString> (roslisp-msg-protocol:ros-message)
  ((CubeDefString
    :reader CubeDefString
    :initarg :CubeDefString
    :type cl:string
    :initform ""))
)

(cl:defclass CubeDefString (<CubeDefString>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CubeDefString>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CubeDefString)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name twophase_solver_ros-msg:<CubeDefString> is deprecated: use twophase_solver_ros-msg:CubeDefString instead.")))

(cl:ensure-generic-function 'CubeDefString-val :lambda-list '(m))
(cl:defmethod CubeDefString-val ((m <CubeDefString>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader twophase_solver_ros-msg:CubeDefString-val is deprecated.  Use twophase_solver_ros-msg:CubeDefString instead.")
  (CubeDefString m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CubeDefString>) ostream)
  "Serializes a message object of type '<CubeDefString>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'CubeDefString))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'CubeDefString))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CubeDefString>) istream)
  "Deserializes a message object of type '<CubeDefString>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CubeDefString) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'CubeDefString) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CubeDefString>)))
  "Returns string type for a message object of type '<CubeDefString>"
  "twophase_solver_ros/CubeDefString")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CubeDefString)))
  "Returns string type for a message object of type 'CubeDefString"
  "twophase_solver_ros/CubeDefString")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CubeDefString>)))
  "Returns md5sum for a message object of type '<CubeDefString>"
  "a3ff4760f41bf547735696b3857691d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CubeDefString)))
  "Returns md5sum for a message object of type 'CubeDefString"
  "a3ff4760f41bf547735696b3857691d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CubeDefString>)))
  "Returns full string definition for message of type '<CubeDefString>"
  (cl:format cl:nil "string CubeDefString~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CubeDefString)))
  "Returns full string definition for message of type 'CubeDefString"
  (cl:format cl:nil "string CubeDefString~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CubeDefString>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'CubeDefString))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CubeDefString>))
  "Converts a ROS message object to a list"
  (cl:list 'CubeDefString
    (cl:cons ':CubeDefString (CubeDefString msg))
))
