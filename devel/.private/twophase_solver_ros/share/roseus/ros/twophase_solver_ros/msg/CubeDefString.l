;; Auto-generated. Do not edit!


(when (boundp 'twophase_solver_ros::CubeDefString)
  (if (not (find-package "TWOPHASE_SOLVER_ROS"))
    (make-package "TWOPHASE_SOLVER_ROS"))
  (shadow 'CubeDefString (find-package "TWOPHASE_SOLVER_ROS")))
(unless (find-package "TWOPHASE_SOLVER_ROS::CUBEDEFSTRING")
  (make-package "TWOPHASE_SOLVER_ROS::CUBEDEFSTRING"))

(in-package "ROS")
;;//! \htmlinclude CubeDefString.msg.html


(defclass twophase_solver_ros::CubeDefString
  :super ros::object
  :slots (_CubeDefString ))

(defmethod twophase_solver_ros::CubeDefString
  (:init
   (&key
    ((:CubeDefString __CubeDefString) "")
    )
   (send-super :init)
   (setq _CubeDefString (string __CubeDefString))
   self)
  (:CubeDefString
   (&optional __CubeDefString)
   (if __CubeDefString (setq _CubeDefString __CubeDefString)) _CubeDefString)
  (:serialization-length
   ()
   (+
    ;; string _CubeDefString
    4 (length _CubeDefString)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _CubeDefString
       (write-long (length _CubeDefString) s) (princ _CubeDefString s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _CubeDefString
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _CubeDefString (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get twophase_solver_ros::CubeDefString :md5sum-) "a3ff4760f41bf547735696b3857691d0")
(setf (get twophase_solver_ros::CubeDefString :datatype-) "twophase_solver_ros/CubeDefString")
(setf (get twophase_solver_ros::CubeDefString :definition-)
      "string CubeDefString

")



(provide :twophase_solver_ros/CubeDefString "a3ff4760f41bf547735696b3857691d0")


