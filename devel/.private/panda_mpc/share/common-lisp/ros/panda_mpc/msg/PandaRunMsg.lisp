; Auto-generated. Do not edit!


(cl:in-package panda_mpc-msg)


;//! \htmlinclude PandaRunMsg.msg.html

(cl:defclass <PandaRunMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (vmax_ec
    :reader vmax_ec
    :initarg :vmax_ec
    :type cl:float
    :initform 0.0)
   (t_traj_curr
    :reader t_traj_curr
    :initarg :t_traj_curr
    :type cl:float
    :initform 0.0)
   (Xd_traj
    :reader Xd_traj
    :initarg :Xd_traj
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (Xdd_traj
    :reader Xdd_traj
    :initarg :Xdd_traj
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (X_err
    :reader X_err
    :initarg :X_err
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (Xd_control
    :reader Xd_control
    :initarg :Xd_control
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (qd_des
    :reader qd_des
    :initarg :qd_des
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState))
   (play_traj_
    :reader play_traj_
    :initarg :play_traj_
    :type cl:boolean
    :initform cl:nil)
   (positioning_
    :reader positioning_
    :initarg :positioning_
    :type cl:boolean
    :initform cl:nil)
   (tune_gains_
    :reader tune_gains_
    :initarg :tune_gains_
    :type cl:boolean
    :initform cl:nil)
   (distance_to_contact_
    :reader distance_to_contact_
    :initarg :distance_to_contact_
    :type cl:float
    :initform 0.0))
)

(cl:defclass PandaRunMsg (<PandaRunMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PandaRunMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PandaRunMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_mpc-msg:<PandaRunMsg> is deprecated: use panda_mpc-msg:PandaRunMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:header-val is deprecated.  Use panda_mpc-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'vmax_ec-val :lambda-list '(m))
(cl:defmethod vmax_ec-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:vmax_ec-val is deprecated.  Use panda_mpc-msg:vmax_ec instead.")
  (vmax_ec m))

(cl:ensure-generic-function 't_traj_curr-val :lambda-list '(m))
(cl:defmethod t_traj_curr-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:t_traj_curr-val is deprecated.  Use panda_mpc-msg:t_traj_curr instead.")
  (t_traj_curr m))

(cl:ensure-generic-function 'Xd_traj-val :lambda-list '(m))
(cl:defmethod Xd_traj-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:Xd_traj-val is deprecated.  Use panda_mpc-msg:Xd_traj instead.")
  (Xd_traj m))

(cl:ensure-generic-function 'Xdd_traj-val :lambda-list '(m))
(cl:defmethod Xdd_traj-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:Xdd_traj-val is deprecated.  Use panda_mpc-msg:Xdd_traj instead.")
  (Xdd_traj m))

(cl:ensure-generic-function 'X_err-val :lambda-list '(m))
(cl:defmethod X_err-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:X_err-val is deprecated.  Use panda_mpc-msg:X_err instead.")
  (X_err m))

(cl:ensure-generic-function 'Xd_control-val :lambda-list '(m))
(cl:defmethod Xd_control-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:Xd_control-val is deprecated.  Use panda_mpc-msg:Xd_control instead.")
  (Xd_control m))

(cl:ensure-generic-function 'qd_des-val :lambda-list '(m))
(cl:defmethod qd_des-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:qd_des-val is deprecated.  Use panda_mpc-msg:qd_des instead.")
  (qd_des m))

(cl:ensure-generic-function 'play_traj_-val :lambda-list '(m))
(cl:defmethod play_traj_-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:play_traj_-val is deprecated.  Use panda_mpc-msg:play_traj_ instead.")
  (play_traj_ m))

(cl:ensure-generic-function 'positioning_-val :lambda-list '(m))
(cl:defmethod positioning_-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:positioning_-val is deprecated.  Use panda_mpc-msg:positioning_ instead.")
  (positioning_ m))

(cl:ensure-generic-function 'tune_gains_-val :lambda-list '(m))
(cl:defmethod tune_gains_-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:tune_gains_-val is deprecated.  Use panda_mpc-msg:tune_gains_ instead.")
  (tune_gains_ m))

(cl:ensure-generic-function 'distance_to_contact_-val :lambda-list '(m))
(cl:defmethod distance_to_contact_-val ((m <PandaRunMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-msg:distance_to_contact_-val is deprecated.  Use panda_mpc-msg:distance_to_contact_ instead.")
  (distance_to_contact_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PandaRunMsg>) ostream)
  "Serializes a message object of type '<PandaRunMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vmax_ec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 't_traj_curr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Xd_traj) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Xdd_traj) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'X_err) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Xd_control) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'qd_des) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'play_traj_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'positioning_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tune_gains_) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance_to_contact_))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PandaRunMsg>) istream)
  "Deserializes a message object of type '<PandaRunMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vmax_ec) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't_traj_curr) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Xd_traj) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Xdd_traj) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'X_err) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Xd_control) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'qd_des) istream)
    (cl:setf (cl:slot-value msg 'play_traj_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'positioning_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tune_gains_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_to_contact_) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PandaRunMsg>)))
  "Returns string type for a message object of type '<PandaRunMsg>"
  "panda_mpc/PandaRunMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PandaRunMsg)))
  "Returns string type for a message object of type 'PandaRunMsg"
  "panda_mpc/PandaRunMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PandaRunMsg>)))
  "Returns md5sum for a message object of type '<PandaRunMsg>"
  "978d838c2f0b7635c26f79d5649e9439")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PandaRunMsg)))
  "Returns md5sum for a message object of type 'PandaRunMsg"
  "978d838c2f0b7635c26f79d5649e9439")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PandaRunMsg>)))
  "Returns full string definition for message of type '<PandaRunMsg>"
  (cl:format cl:nil "Header header~%float64 vmax_ec # [m/s]~%float64 t_traj_curr~%geometry_msgs/Twist Xd_traj~%geometry_msgs/Twist Xdd_traj~%geometry_msgs/Twist X_err~%geometry_msgs/Twist Xd_control~%sensor_msgs/JointState qd_des~%bool play_traj_~%bool positioning_~%bool tune_gains_~%float64 distance_to_contact_~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PandaRunMsg)))
  "Returns full string definition for message of type 'PandaRunMsg"
  (cl:format cl:nil "Header header~%float64 vmax_ec # [m/s]~%float64 t_traj_curr~%geometry_msgs/Twist Xd_traj~%geometry_msgs/Twist Xdd_traj~%geometry_msgs/Twist X_err~%geometry_msgs/Twist Xd_control~%sensor_msgs/JointState qd_des~%bool play_traj_~%bool positioning_~%bool tune_gains_~%float64 distance_to_contact_~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PandaRunMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Xd_traj))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Xdd_traj))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'X_err))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Xd_control))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'qd_des))
     1
     1
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PandaRunMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PandaRunMsg
    (cl:cons ':header (header msg))
    (cl:cons ':vmax_ec (vmax_ec msg))
    (cl:cons ':t_traj_curr (t_traj_curr msg))
    (cl:cons ':Xd_traj (Xd_traj msg))
    (cl:cons ':Xdd_traj (Xdd_traj msg))
    (cl:cons ':X_err (X_err msg))
    (cl:cons ':Xd_control (Xd_control msg))
    (cl:cons ':qd_des (qd_des msg))
    (cl:cons ':play_traj_ (play_traj_ msg))
    (cl:cons ':positioning_ (positioning_ msg))
    (cl:cons ':tune_gains_ (tune_gains_ msg))
    (cl:cons ':distance_to_contact_ (distance_to_contact_ msg))
))
