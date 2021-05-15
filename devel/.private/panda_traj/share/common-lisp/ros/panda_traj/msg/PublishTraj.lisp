; Auto-generated. Do not edit!


(cl:in-package panda_traj-msg)


;//! \htmlinclude PublishTraj.msg.html

(cl:defclass <PublishTraj> (roslisp-msg-protocol:ros-message)
  ((pose_array_
    :reader pose_array_
    :initarg :pose_array_
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray))
   (path_ros_
    :reader path_ros_
    :initarg :path_ros_
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass PublishTraj (<PublishTraj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PublishTraj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PublishTraj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_traj-msg:<PublishTraj> is deprecated: use panda_traj-msg:PublishTraj instead.")))

(cl:ensure-generic-function 'pose_array_-val :lambda-list '(m))
(cl:defmethod pose_array_-val ((m <PublishTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-msg:pose_array_-val is deprecated.  Use panda_traj-msg:pose_array_ instead.")
  (pose_array_ m))

(cl:ensure-generic-function 'path_ros_-val :lambda-list '(m))
(cl:defmethod path_ros_-val ((m <PublishTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-msg:path_ros_-val is deprecated.  Use panda_traj-msg:path_ros_ instead.")
  (path_ros_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PublishTraj>) ostream)
  "Serializes a message object of type '<PublishTraj>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_array_) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path_ros_) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PublishTraj>) istream)
  "Deserializes a message object of type '<PublishTraj>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_array_) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path_ros_) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PublishTraj>)))
  "Returns string type for a message object of type '<PublishTraj>"
  "panda_traj/PublishTraj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PublishTraj)))
  "Returns string type for a message object of type 'PublishTraj"
  "panda_traj/PublishTraj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PublishTraj>)))
  "Returns md5sum for a message object of type '<PublishTraj>"
  "23616706435ed56017355b3bc7eafc99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PublishTraj)))
  "Returns md5sum for a message object of type 'PublishTraj"
  "23616706435ed56017355b3bc7eafc99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PublishTraj>)))
  "Returns full string definition for message of type '<PublishTraj>"
  (cl:format cl:nil "geometry_msgs/PoseArray pose_array_~%nav_msgs/Path path_ros_~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PublishTraj)))
  "Returns full string definition for message of type 'PublishTraj"
  (cl:format cl:nil "geometry_msgs/PoseArray pose_array_~%nav_msgs/Path path_ros_~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PublishTraj>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_array_))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path_ros_))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PublishTraj>))
  "Converts a ROS message object to a list"
  (cl:list 'PublishTraj
    (cl:cons ':pose_array_ (pose_array_ msg))
    (cl:cons ':path_ros_ (path_ros_ msg))
))
