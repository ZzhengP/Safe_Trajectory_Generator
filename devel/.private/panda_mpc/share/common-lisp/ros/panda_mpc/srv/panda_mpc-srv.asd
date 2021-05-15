
(cl:in-package :asdf)

(defsystem "panda_mpc-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "UI" :depends-on ("_package_UI"))
    (:file "_package_UI" :depends-on ("_package"))
  ))