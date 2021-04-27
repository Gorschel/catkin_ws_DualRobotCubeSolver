;; Auto-generated. Do not edit!


(when (boundp 'twophase_solver_ros::SolveMsg)
  (if (not (find-package "TWOPHASE_SOLVER_ROS"))
    (make-package "TWOPHASE_SOLVER_ROS"))
  (shadow 'SolveMsg (find-package "TWOPHASE_SOLVER_ROS")))
(unless (find-package "TWOPHASE_SOLVER_ROS::SOLVEMSG")
  (make-package "TWOPHASE_SOLVER_ROS::SOLVEMSG"))

(in-package "ROS")
;;//! \htmlinclude SolveMsg.msg.html


(defclass twophase_solver_ros::SolveMsg
  :super ros::object
  :slots (_movecount _SolveString ))

(defmethod twophase_solver_ros::SolveMsg
  (:init
   (&key
    ((:movecount __movecount) 0)
    ((:SolveString __SolveString) "")
    )
   (send-super :init)
   (setq _movecount (round __movecount))
   (setq _SolveString (string __SolveString))
   self)
  (:movecount
   (&optional __movecount)
   (if __movecount (setq _movecount __movecount)) _movecount)
  (:SolveString
   (&optional __SolveString)
   (if __SolveString (setq _SolveString __SolveString)) _SolveString)
  (:serialization-length
   ()
   (+
    ;; int64 _movecount
    8
    ;; string _SolveString
    4 (length _SolveString)
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
     ;; string _SolveString
       (write-long (length _SolveString) s) (princ _SolveString s)
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
   ;; string _SolveString
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _SolveString (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get twophase_solver_ros::SolveMsg :md5sum-) "f8b96427a91926b7a010189fd3281ee2")
(setf (get twophase_solver_ros::SolveMsg :datatype-) "twophase_solver_ros/SolveMsg")
(setf (get twophase_solver_ros::SolveMsg :definition-)
      "int64 movecount
string SolveString

")



(provide :twophase_solver_ros/SolveMsg "f8b96427a91926b7a010189fd3281ee2")


