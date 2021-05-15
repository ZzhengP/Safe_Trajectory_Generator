
(cl:in-package :asdf)

(defsystem "panda_traj-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "UpdateTrajectory" :depends-on ("_package_UpdateTrajectory"))
    (:file "_package_UpdateTrajectory" :depends-on ("_package"))
  ))