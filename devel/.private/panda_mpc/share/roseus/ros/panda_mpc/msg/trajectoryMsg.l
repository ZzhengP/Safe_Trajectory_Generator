;; Auto-generated. Do not edit!


(when (boundp 'panda_mpc::trajectoryMsg)
  (if (not (find-package "PANDA_MPC"))
    (make-package "PANDA_MPC"))
  (shadow 'trajectoryMsg (find-package "PANDA_MPC")))
(unless (find-package "PANDA_MPC::TRAJECTORYMSG")
  (make-package "PANDA_MPC::TRAJECTORYMSG"))

(in-package "ROS")
;;//! \htmlinclude trajectoryMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass panda_mpc::trajectoryMsg
  :super ros::object
  :slots (_header _next_point _next_vel ))

(defmethod panda_mpc::trajectoryMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:next_point __next_point) (instance geometry_msgs::Vector3 :init))
    ((:next_vel __next_vel) (instance geometry_msgs::Twist :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _next_point __next_point)
   (setq _next_vel __next_vel)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:next_point
   (&rest __next_point)
   (if (keywordp (car __next_point))
       (send* _next_point __next_point)
     (progn
       (if __next_point (setq _next_point (car __next_point)))
       _next_point)))
  (:next_vel
   (&rest __next_vel)
   (if (keywordp (car __next_vel))
       (send* _next_vel __next_vel)
     (progn
       (if __next_vel (setq _next_vel (car __next_vel)))
       _next_vel)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _next_point
    (send _next_point :serialization-length)
    ;; geometry_msgs/Twist _next_vel
    (send _next_vel :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _next_point
       (send _next_point :serialize s)
     ;; geometry_msgs/Twist _next_vel
       (send _next_vel :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _next_point
     (send _next_point :deserialize buf ptr-) (incf ptr- (send _next_point :serialization-length))
   ;; geometry_msgs/Twist _next_vel
     (send _next_vel :deserialize buf ptr-) (incf ptr- (send _next_vel :serialization-length))
   ;;
   self)
  )

(setf (get panda_mpc::trajectoryMsg :md5sum-) "87e2c8c7845fec4f6ddc003e2af217c5")
(setf (get panda_mpc::trajectoryMsg :datatype-) "panda_mpc/trajectoryMsg")
(setf (get panda_mpc::trajectoryMsg :definition-)
      "Header header
geometry_msgs/Vector3 next_point
geometry_msgs/Twist next_vel

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

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
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

")



(provide :panda_mpc/trajectoryMsg "87e2c8c7845fec4f6ddc003e2af217c5")


