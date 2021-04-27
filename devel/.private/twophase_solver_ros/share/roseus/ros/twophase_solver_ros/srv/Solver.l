;; Auto-generated. Do not edit!


(when (boundp 'twophase_solver_ros::Solver)
  (if (not (find-package "TWOPHASE_SOLVER_ROS"))
    (make-package "TWOPHASE_SOLVER_ROS"))
  (shadow 'Solver (find-package "TWOPHASE_SOLVER_ROS")))
(unless (find-package "TWOPHASE_SOLVER_ROS::SOLVER")
  (make-package "TWOPHASE_SOLVER_ROS::SOLVER"))
(unless (find-package "TWOPHASE_SOLVER_ROS::SOLVERREQUEST")
  (make-package "TWOPHASE_SOLVER_ROS::SOLVERREQUEST"))
(unless (find-package "TWOPHASE_SOLVER_ROS::SOLVERRESPONSE")
  (make-package "TWOPHASE_SOLVER_ROS::SOLVERRESPONSE"))

(in-package "ROS")





(defclass twophase_solver_ros::SolverRequest
  :super ros::object
  :slots (_defstr ))

(defmethod twophase_solver_ros::SolverRequest
  (:init
   (&key
    ((:defstr __defstr) "")
    )
   (send-super :init)
   (setq _defstr (string __defstr))
   self)
  (:defstr
   (&optional __defstr)
   (if __defstr (setq _defstr __defstr)) _defstr)
  (:serialization-length
   ()
   (+
    ;; string _defstr
    4 (length _defstr)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _defstr
       (write-long (length _defstr) s) (princ _defstr s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _defstr
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _defstr (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass twophase_solver_ros::SolverResponse
  :super ros::object
  :slots (_movecount _solution ))

(defmethod twophase_solver_ros::SolverResponse
  (:init
   (&key
    ((:movecount __movecount) 0)
    ((:solution __solution) "")
    )
   (send-super :init)
   (setq _movecount (round __movecount))
   (setq _solution (string __solution))
   self)
  (:movecount
   (&optional __movecount)
   (if __movecount (setq _movecount __movecount)) _movecount)
  (:solution
   (&optional __solution)
   (if __solution (setq _solution __solution)) _solution)
  (:serialization-length
   ()
   (+
    ;; int64 _movecount
    8
    ;; string _solution
    4 (length _solution)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _movecount
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _movecount (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _movecount) (= (length (_movecount . bv)) 2)) ;; bignum
              (write-long (ash (elt (_movecount . bv) 0) 0) s)
              (write-long (ash (elt (_movecount . bv) 1) -1) s))
             ((and (class _movecount) (= (length (_movecount . bv)) 1)) ;; big1
              (write-long (elt (_movecount . bv) 0) s)
              (write-long (if (>= _movecount 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _movecount s)(write-long (if (>= _movecount 0) 0 #xffffffff) s)))
     ;; string _solution
       (write-long (length _solution) s) (princ _solution s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _movecount
#+(or :alpha :irix6 :x86_64)
      (setf _movecount (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _movecount (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _solution
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _solution (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass twophase_solver_ros::Solver
  :super ros::object
  :slots ())

(setf (get twophase_solver_ros::Solver :md5sum-) "3dd62463a4927de8181123eea48c8934")
(setf (get twophase_solver_ros::Solver :datatype-) "twophase_solver_ros/Solver")
(setf (get twophase_solver_ros::Solver :request) twophase_solver_ros::SolverRequest)
(setf (get twophase_solver_ros::Solver :response) twophase_solver_ros::SolverResponse)

(defmethod twophase_solver_ros::SolverRequest
  (:response () (instance twophase_solver_ros::SolverResponse :init)))

(setf (get twophase_solver_ros::SolverRequest :md5sum-) "3dd62463a4927de8181123eea48c8934")
(setf (get twophase_solver_ros::SolverRequest :datatype-) "twophase_solver_ros/SolverRequest")
(setf (get twophase_solver_ros::SolverRequest :definition-)
      "string defstr
---
int64 movecount
string solution

")

(setf (get twophase_solver_ros::SolverResponse :md5sum-) "3dd62463a4927de8181123eea48c8934")
(setf (get twophase_solver_ros::SolverResponse :datatype-) "twophase_solver_ros/SolverResponse")
(setf (get twophase_solver_ros::SolverResponse :definition-)
      "string defstr
---
int64 movecount
string solution

")



(provide :twophase_solver_ros/Solver "3dd62463a4927de8181123eea48c8934")


