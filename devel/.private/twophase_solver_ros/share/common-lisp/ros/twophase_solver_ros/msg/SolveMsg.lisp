; Auto-generated. Do not edit!


(cl:in-package twophase_solver_ros-msg)


;//! \htmlinclude SolveMsg.msg.html

(cl:defclass <SolveMsg> (roslisp-msg-protocol:ros-message)
  ((movecount
    :reader movecount
    :initarg :movecount
    :type cl:integer
    :initform 0)
   (SolveString
    :reader SolveString
    :initarg :SolveString
    :type cl:string
    :initform ""))
)

(cl:defclass SolveMsg (<SolveMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SolveMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SolveMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name twophase_solver_ros-msg:<SolveMsg> is deprecated: use twophase_solver_ros-msg:SolveMsg instead.")))

(cl:ensure-generic-function 'movecount-val :lambda-list '(m))
(cl:defmethod movecount-val ((m <SolveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader twophase_solver_ros-msg:movecount-val is deprecated.  Use twophase_solver_ros-msg:movecount instead.")
  (movecount m))

(cl:ensure-generic-function 'SolveString-val :lambda-list '(m))
(cl:defmethod SolveString-val ((m <SolveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader twophase_solver_ros-msg:SolveString-val is deprecated.  Use twophase_solver_ros-msg:SolveString instead.")
  (SolveString m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SolveMsg>) ostream)
  "Serializes a message object of type '<SolveMsg>"
  (cl:let* ((signed (cl:slot-value msg 'movecount)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'SolveString))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'SolveString))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SolveMsg>) istream)
  "Deserializes a message object of type '<SolveMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'movecount) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'SolveString) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'SolveString) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SolveMsg>)))
  "Returns string type for a message object of type '<SolveMsg>"
  "twophase_solver_ros/SolveMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SolveMsg)))
  "Returns string type for a message object of type 'SolveMsg"
  "twophase_solver_ros/SolveMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SolveMsg>)))
  "Returns md5sum for a message object of type '<SolveMsg>"
  "f8b96427a91926b7a010189fd3281ee2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SolveMsg)))
  "Returns md5sum for a message object of type 'SolveMsg"
  "f8b96427a91926b7a010189fd3281ee2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SolveMsg>)))
  "Returns full string definition for message of type '<SolveMsg>"
  (cl:format cl:nil "int64 movecount~%string SolveString~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SolveMsg)))
  "Returns full string definition for message of type 'SolveMsg"
  (cl:format cl:nil "int64 movecount~%string SolveString~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SolveMsg>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'SolveString))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SolveMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'SolveMsg
    (cl:cons ':movecount (movecount msg))
    (cl:cons ':SolveString (SolveString msg))
))
