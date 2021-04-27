; Auto-generated. Do not edit!


(cl:in-package twophase_solver_ros-srv)


;//! \htmlinclude Solver-request.msg.html

(cl:defclass <Solver-request> (roslisp-msg-protocol:ros-message)
  ((defstr
    :reader defstr
    :initarg :defstr
    :type cl:string
    :initform ""))
)

(cl:defclass Solver-request (<Solver-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Solver-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Solver-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name twophase_solver_ros-srv:<Solver-request> is deprecated: use twophase_solver_ros-srv:Solver-request instead.")))

(cl:ensure-generic-function 'defstr-val :lambda-list '(m))
(cl:defmethod defstr-val ((m <Solver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader twophase_solver_ros-srv:defstr-val is deprecated.  Use twophase_solver_ros-srv:defstr instead.")
  (defstr m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Solver-request>) ostream)
  "Serializes a message object of type '<Solver-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'defstr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'defstr))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Solver-request>) istream)
  "Deserializes a message object of type '<Solver-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'defstr) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'defstr) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Solver-request>)))
  "Returns string type for a service object of type '<Solver-request>"
  "twophase_solver_ros/SolverRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Solver-request)))
  "Returns string type for a service object of type 'Solver-request"
  "twophase_solver_ros/SolverRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Solver-request>)))
  "Returns md5sum for a message object of type '<Solver-request>"
  "3dd62463a4927de8181123eea48c8934")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Solver-request)))
  "Returns md5sum for a message object of type 'Solver-request"
  "3dd62463a4927de8181123eea48c8934")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Solver-request>)))
  "Returns full string definition for message of type '<Solver-request>"
  (cl:format cl:nil "string defstr~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Solver-request)))
  "Returns full string definition for message of type 'Solver-request"
  (cl:format cl:nil "string defstr~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Solver-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'defstr))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Solver-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Solver-request
    (cl:cons ':defstr (defstr msg))
))
;//! \htmlinclude Solver-response.msg.html

(cl:defclass <Solver-response> (roslisp-msg-protocol:ros-message)
  ((movecount
    :reader movecount
    :initarg :movecount
    :type cl:integer
    :initform 0)
   (solution
    :reader solution
    :initarg :solution
    :type cl:string
    :initform ""))
)

(cl:defclass Solver-response (<Solver-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Solver-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Solver-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name twophase_solver_ros-srv:<Solver-response> is deprecated: use twophase_solver_ros-srv:Solver-response instead.")))

(cl:ensure-generic-function 'movecount-val :lambda-list '(m))
(cl:defmethod movecount-val ((m <Solver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader twophase_solver_ros-srv:movecount-val is deprecated.  Use twophase_solver_ros-srv:movecount instead.")
  (movecount m))

(cl:ensure-generic-function 'solution-val :lambda-list '(m))
(cl:defmethod solution-val ((m <Solver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader twophase_solver_ros-srv:solution-val is deprecated.  Use twophase_solver_ros-srv:solution instead.")
  (solution m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Solver-response>) ostream)
  "Serializes a message object of type '<Solver-response>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'solution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'solution))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Solver-response>) istream)
  "Deserializes a message object of type '<Solver-response>"
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
      (cl:setf (cl:slot-value msg 'solution) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'solution) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Solver-response>)))
  "Returns string type for a service object of type '<Solver-response>"
  "twophase_solver_ros/SolverResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Solver-response)))
  "Returns string type for a service object of type 'Solver-response"
  "twophase_solver_ros/SolverResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Solver-response>)))
  "Returns md5sum for a message object of type '<Solver-response>"
  "3dd62463a4927de8181123eea48c8934")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Solver-response)))
  "Returns md5sum for a message object of type 'Solver-response"
  "3dd62463a4927de8181123eea48c8934")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Solver-response>)))
  "Returns full string definition for message of type '<Solver-response>"
  (cl:format cl:nil "int64 movecount~%string solution~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Solver-response)))
  "Returns full string definition for message of type 'Solver-response"
  (cl:format cl:nil "int64 movecount~%string solution~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Solver-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'solution))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Solver-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Solver-response
    (cl:cons ':movecount (movecount msg))
    (cl:cons ':solution (solution msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Solver)))
  'Solver-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Solver)))
  'Solver-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Solver)))
  "Returns string type for a service object of type '<Solver>"
  "twophase_solver_ros/Solver")