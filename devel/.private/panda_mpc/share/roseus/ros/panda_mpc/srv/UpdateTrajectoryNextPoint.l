;; Auto-generated. Do not edit!


(when (boundp 'panda_mpc::UpdateTrajectoryNextPoint)
  (if (not (find-package "PANDA_MPC"))
    (make-package "PANDA_MPC"))
  (shadow 'UpdateTrajectoryNextPoint (find-package "PANDA_MPC")))
(unless (find-package "PANDA_MPC::UPDATETRAJECTORYNEXTPOINT")
  (make-package "PANDA_MPC::UPDATETRAJECTORYNEXTPOINT"))
(unless (find-package "PANDA_MPC::UPDATETRAJECTORYNEXTPOINTREQUEST")
  (make-package "PANDA_MPC::UPDATETRAJECTORYNEXTPOINTREQUEST"))
(unless (find-package "PANDA_MPC::UPDATETRAJECTORYNEXTPOINTRESPONSE")
  (make-package "PANDA_MPC::UPDATETRAJECTORYNEXTPOINTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass panda_mpc::UpdateTrajectoryNextPointRequest
  :super ros::object
  :slots (_next_point _vel ))

(defmethod panda_mpc::UpdateTrajectoryNextPointRequest
  (:init
   (&key
    ((:next_point __next_point) (instance geometry_msgs::Twist :init))
    ((:vel __vel) 0.0)
    )
   (send-super :init)
   (setq _next_point __next_point)
   (setq _vel (float __vel))
   self)
  (:next_point
   (&rest __next_point)
   (if (keywordp (car __next_point))
       (send* _next_point __next_point)
     (progn
       (if __next_point (setq _next_point (car __next_point)))
       _next_point)))
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Twist _next_point
    (send _next_point :serialization-length)
    ;; float64 _vel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Twist _next_point
       (send _next_point :serialize s)
     ;; float64 _vel
       (sys::poke _vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Twist _next_point
     (send _next_point :deserialize buf ptr-) (incf ptr- (send _next_point :serialization-length))
   ;; float64 _vel
     (setq _vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass panda_mpc::UpdateTrajectoryNextPointResponse
  :super ros::object
  :slots (_success ))

(defmethod panda_mpc::UpdateTrajectoryNextPointResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass panda_mpc::UpdateTrajectoryNextPoint
  :super ros::object
  :slots ())

(setf (get panda_mpc::UpdateTrajectoryNextPoint :md5sum-) "2134cf5914ee56390a99d4f1731a484b")
(setf (get panda_mpc::UpdateTrajectoryNextPoint :datatype-) "panda_mpc/UpdateTrajectoryNextPoint")
(setf (get panda_mpc::UpdateTrajectoryNextPoint :request) panda_mpc::UpdateTrajectoryNextPointRequest)
(setf (get panda_mpc::UpdateTrajectoryNextPoint :response) panda_mpc::UpdateTrajectoryNextPointResponse)

(defmethod panda_mpc::UpdateTrajectoryNextPointRequest
  (:response () (instance panda_mpc::UpdateTrajectoryNextPointResponse :init)))

(setf (get panda_mpc::UpdateTrajectoryNextPointRequest :md5sum-) "2134cf5914ee56390a99d4f1731a484b")
(setf (get panda_mpc::UpdateTrajectoryNextPointRequest :datatype-) "panda_mpc/UpdateTrajectoryNextPointRequest")
(setf (get panda_mpc::UpdateTrajectoryNextPointRequest :definition-)
      "geometry_msgs/Twist next_point
float64 vel

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool success

")

(setf (get panda_mpc::UpdateTrajectoryNextPointResponse :md5sum-) "2134cf5914ee56390a99d4f1731a484b")
(setf (get panda_mpc::UpdateTrajectoryNextPointResponse :datatype-) "panda_mpc/UpdateTrajectoryNextPointResponse")
(setf (get panda_mpc::UpdateTrajectoryNextPointResponse :definition-)
      "geometry_msgs/Twist next_point
float64 vel

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool success

")



(provide :panda_mpc/UpdateTrajectoryNextPoint "2134cf5914ee56390a99d4f1731a484b")


