
(cl:in-package :asdf)

(defsystem "twophase_solver_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Solver" :depends-on ("_package_Solver"))
    (:file "_package_Solver" :depends-on ("_package"))
  ))