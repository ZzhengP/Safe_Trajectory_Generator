
(cl:in-package :asdf)

(defsystem "panda_mpc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointTorqueComparison" :depends-on ("_package_JointTorqueComparison"))
    (:file "_package_JointTorqueComparison" :depends-on ("_package"))
    (:file "PandaRunMsg" :depends-on ("_package_PandaRunMsg"))
    (:file "_package_PandaRunMsg" :depends-on ("_package"))
    (:file "trajectoryMsg" :depends-on ("_package_trajectoryMsg"))
    (:file "_package_trajectoryMsg" :depends-on ("_package"))
  ))