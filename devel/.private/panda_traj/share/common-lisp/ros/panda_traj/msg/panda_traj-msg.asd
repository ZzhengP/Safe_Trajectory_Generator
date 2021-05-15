
(cl:in-package :asdf)

(defsystem "panda_traj-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "PublishTraj" :depends-on ("_package_PublishTraj"))
    (:file "_package_PublishTraj" :depends-on ("_package"))
    (:file "TrajProperties" :depends-on ("_package_TrajProperties"))
    (:file "_package_TrajProperties" :depends-on ("_package"))
  ))