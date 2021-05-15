;; Auto-generated. Do not edit!


(when (boundp 'panda_traj::PublishTraj)
  (if (not (find-package "PANDA_TRAJ"))
    (make-package "PANDA_TRAJ"))
  (shadow 'PublishTraj (find-package "PANDA_TRAJ")))
(unless (find-package "PANDA_TRAJ::PUBLISHTRAJ")
  (make-package "PANDA_TRAJ::PUBLISHTRAJ"))

(in-package "ROS")
;;//! \htmlinclude PublishTraj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass panda_traj::PublishTraj
  :super ros::object
  :slots (_pose_array_ _path_ros_ ))

(defmethod panda_traj::PublishTraj
  (:init
   (&key
    ((:pose_array_ __pose_array_) (instance geometry_msgs::PoseArray :init))
    ((:path_ros_ __path_ros_) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _pose_array_ __pose_array_)
   (setq _path_ros_ __path_ros_)
   self)
  (:pose_array_
   (&rest __pose_array_)
   (if (keywordp (car __pose_array_))
       (send* _pose_array_ __pose_array_)
     (progn
       (if __pose_array_ (setq _pose_array_ (car __pose_array_)))
       _pose_array_)))
  (:path_ros_
   (&rest __path_ros_)
   (if (keywordp (car __path_ros_))
       (send* _path_ros_ __path_ros_)
     (progn
       (if __path_ros_ (setq _path_ros_ (car __path_ros_)))
       _path_ros_)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseArray _pose_array_
    (send _pose_array_ :serialization-length)
    ;; nav_msgs/Path _path_ros_
    (send _path_ros_ :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseArray _pose_array_
       (send _pose_array_ :serialize s)
     ;; nav_msgs/Path _path_ros_
       (send _path_ros_ :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseArray _pose_array_
     (send _pose_array_ :deserialize buf ptr-) (incf ptr- (send _pose_array_ :serialization-length))
   ;; nav_msgs/Path _path_ros_
     (send _path_ros_ :deserialize buf ptr-) (incf ptr- (send _path_ros_ :serialization-length))
   ;;
   self)
  )

(setf (get panda_traj::PublishTraj :md5sum-) "23616706435ed56017355b3bc7eafc99")
(setf (get panda_traj::PublishTraj :datatype-) "panda_traj/PublishTraj")
(setf (get panda_traj::PublishTraj :definition-)
      "geometry_msgs/PoseArray pose_array_
nav_msgs/Path path_ros_
================================================================================
MSG: geometry_msgs/PoseArray
# An array of poses with a header for global reference.

Header header

Pose[] poses

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

")



(provide :panda_traj/PublishTraj "23616706435ed56017355b3bc7eafc99")


