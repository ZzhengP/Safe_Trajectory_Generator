; Auto-generated. Do not edit!


(cl:in-package panda_mpc-msg)


;//! \htmlinclude trajectoryMsg.msg.html

(cl:defclass <trajectoryMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (next_point
    :reader next_point
    :initarg :next_point
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (next_vel
    :reader next_vel
    :initarg :next_vel
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist)))
)

(cl:defclass trajectoryMsg (<trajectoryMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectoryMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectoryMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_mpc-msg:<trajectoryMsg> is deprecated: use panda_mpc-msg:trajectoryMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <trajectoryMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:header-val is deprecated.  Use panda_mpc-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'next_point-val :lambda-list '(m))
(cl:defmethod next_point-val ((m <trajectoryMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:next_point-val is deprecated.  Use panda_mpc-msg:next_point instead.")
  (next_point m))

(cl:ensure-generic-function 'next_vel-val :lambda-list '(m))
(cl:defmethod next_vel-val ((m <trajectoryMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:next_vel-val is deprecated.  Use panda_mpc-msg:next_vel instead.")
  (next_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectoryMsg>) ostream)
  "Serializes a message object of type '<trajectoryMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'next_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'next_vel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectoryMsg>) istream)
  "Deserializes a message object of type '<trajectoryMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'next_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'next_vel) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectoryMsg>)))
  "Returns string type for a message object of type '<trajectoryMsg>"
  "panda_mpc/trajectoryMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectoryMsg)))
  "Returns string type for a message object of type 'trajectoryMsg"
  "panda_mpc/trajectoryMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectoryMsg>)))
  "Returns md5sum for a message object of type '<trajectoryMsg>"
  "87e2c8c7845fec4f6ddc003e2af217c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectoryMsg)))
  "Returns md5sum for a message object of type 'trajectoryMsg"
  "87e2c8c7845fec4f6ddc003e2af217c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectoryMsg>)))
  "Returns full string definition for message of type '<trajectoryMsg>"
  (cl:format cl:nil "Header header~%geometry_msgs/Vector3 next_point~%geometry_msgs/Twist next_vel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectoryMsg)))
  "Returns full string definition for message of type 'trajectoryMsg"
  (cl:format cl:nil "Header header~%geometry_msgs/Vector3 next_point~%geometry_msgs/Twist next_vel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectoryMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'next_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'next_vel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectoryMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectoryMsg
    (cl:cons ':header (header msg))
    (cl:cons ':next_point (next_point msg))
    (cl:cons ':next_vel (next_vel msg))
))
