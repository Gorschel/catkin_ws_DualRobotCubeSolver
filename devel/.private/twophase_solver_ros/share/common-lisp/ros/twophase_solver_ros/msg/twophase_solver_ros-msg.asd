
(cl:in-package :asdf)

(defsystem "twophase_solver_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CubeDefString" :depends-on ("_package_CubeDefString"))
    (:file "_package_CubeDefString" :depends-on ("_package"))
    (:file "SolveMsg" :depends-on ("_package_SolveMsg"))
    (:file "_package_SolveMsg" :depends-on ("_package"))
  ))