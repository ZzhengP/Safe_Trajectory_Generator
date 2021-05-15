; Auto-generated. Do not edit!


(cl:in-package panda_mpc-srv)


;//! \htmlinclude UI-request.msg.html

(cl:defclass <UI-request> (roslisp-msg-protocol:ros-message)
  ((play_traj
    :reader play_traj
    :initarg :play_traj
    :type cl:boolean
    :initform cl:nil)
   (jog_robot
    :reader jog_robot
    :initarg :jog_robot
    :type cl:boolean
    :initform cl:nil)
   (publish_traj
    :reader publish_traj
    :initarg :publish_traj
    :type cl:boolean
    :initform cl:nil)
   (build_traj
    :reader build_traj
    :initarg :build_traj
    :type cl:boolean
    :initform cl:nil)
   (positioning_
    :reader positioning_
    :initarg :positioning_
    :type cl:boolean
    :initform cl:nil)
   (p_gains_
    :reader p_gains_
    :initarg :p_gains_
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (d_gains_
    :reader d_gains_
    :initarg :d_gains_
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (move_signal_
    :reader move_signal_
    :initarg :move_signal_
    :type cl:boolean
    :initform cl:nil)
   (tune_gain
    :reader tune_gain
    :initarg :tune_gain
    :type cl:boolean
    :initform cl:nil)
   (amplitude
    :reader amplitude
    :initarg :amplitude
    :type cl:float
    :initform 0.0)
   (axis
    :reader axis
    :initarg :axis
    :type cl:integer
    :initform 0)
   (exit_
    :reader exit_
    :initarg :exit_
    :type cl:boolean
    :initform cl:nil)
   (distance_to_contact_
    :reader distance_to_contact_
    :initarg :distance_to_contact_
    :type cl:float
    :initform 0.0)
   (fake_distance_
    :reader fake_distance_
    :initarg :fake_distance_
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UI-request (<UI-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UI-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UI-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_mpc-srv:<UI-request> is deprecated: use panda_mpc-srv:UI-request instead.")))

(cl:ensure-generic-function 'play_traj-val :lambda-list '(m))
(cl:defmethod play_traj-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:play_traj-val is deprecated.  Use panda_mpc-srv:play_traj instead.")
  (play_traj m))

(cl:ensure-generic-function 'jog_robot-val :lambda-list '(m))
(cl:defmethod jog_robot-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:jog_robot-val is deprecated.  Use panda_mpc-srv:jog_robot instead.")
  (jog_robot m))

(cl:ensure-generic-function 'publish_traj-val :lambda-list '(m))
(cl:defmethod publish_traj-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:publish_traj-val is deprecated.  Use panda_mpc-srv:publish_traj instead.")
  (publish_traj m))

(cl:ensure-generic-function 'build_traj-val :lambda-list '(m))
(cl:defmethod build_traj-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:build_traj-val is deprecated.  Use panda_mpc-srv:build_traj instead.")
  (build_traj m))

(cl:ensure-generic-function 'positioning_-val :lambda-list '(m))
(cl:defmethod positioning_-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:positioning_-val is deprecated.  Use panda_mpc-srv:positioning_ instead.")
  (positioning_ m))

(cl:ensure-generic-function 'p_gains_-val :lambda-list '(m))
(cl:defmethod p_gains_-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:p_gains_-val is deprecated.  Use panda_mpc-srv:p_gains_ instead.")
  (p_gains_ m))

(cl:ensure-generic-function 'd_gains_-val :lambda-list '(m))
(cl:defmethod d_gains_-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:d_gains_-val is deprecated.  Use panda_mpc-srv:d_gains_ instead.")
  (d_gains_ m))

(cl:ensure-generic-function 'move_signal_-val :lambda-list '(m))
(cl:defmethod move_signal_-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:move_signal_-val is deprecated.  Use panda_mpc-srv:move_signal_ instead.")
  (move_signal_ m))

(cl:ensure-generic-function 'tune_gain-val :lambda-list '(m))
(cl:defmethod tune_gain-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:tune_gain-val is deprecated.  Use panda_mpc-srv:tune_gain instead.")
  (tune_gain m))

(cl:ensure-generic-function 'amplitude-val :lambda-list '(m))
(cl:defmethod amplitude-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:amplitude-val is deprecated.  Use panda_mpc-srv:amplitude instead.")
  (amplitude m))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:axis-val is deprecated.  Use panda_mpc-srv:axis instead.")
  (axis m))

(cl:ensure-generic-function 'exit_-val :lambda-list '(m))
(cl:defmethod exit_-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:exit_-val is deprecated.  Use panda_mpc-srv:exit_ instead.")
  (exit_ m))

(cl:ensure-generic-function 'distance_to_contact_-val :lambda-list '(m))
(cl:defmethod distance_to_contact_-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:distance_to_contact_-val is deprecated.  Use panda_mpc-srv:distance_to_contact_ instead.")
  (distance_to_contact_ m))

(cl:ensure-generic-function 'fake_distance_-val :lambda-list '(m))
(cl:defmethod fake_distance_-val ((m <UI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:fake_distance_-val is deprecated.  Use panda_mpc-srv:fake_distance_ instead.")
  (fake_distance_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UI-request>) ostream)
  "Serializes a message object of type '<UI-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'play_traj) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'jog_robot) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'publish_traj) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'build_traj) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'positioning_) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p_gains_) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'd_gains_) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'move_signal_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tune_gain) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'amplitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'axis)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'exit_) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance_to_contact_))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fake_distance_) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UI-request>) istream)
  "Deserializes a message object of type '<UI-request>"
    (cl:setf (cl:slot-value msg 'play_traj) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'jog_robot) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'publish_traj) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'build_traj) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'positioning_) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p_gains_) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'd_gains_) istream)
    (cl:setf (cl:slot-value msg 'move_signal_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tune_gain) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amplitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'axis) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:setf (cl:slot-value msg 'exit_) (cl:not (cl:zerop (cl:read-byte istream))))
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
    (cl:setf (cl:slot-value msg 'fake_distance_) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UI-request>)))
  "Returns string type for a service object of type '<UI-request>"
  "panda_mpc/UIRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UI-request)))
  "Returns string type for a service object of type 'UI-request"
  "panda_mpc/UIRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UI-request>)))
  "Returns md5sum for a message object of type '<UI-request>"
  "2e1bdac3ef57deb56fae70ed4385f616")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UI-request)))
  "Returns md5sum for a message object of type 'UI-request"
  "2e1bdac3ef57deb56fae70ed4385f616")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UI-request>)))
  "Returns full string definition for message of type '<UI-request>"
  (cl:format cl:nil "bool play_traj~%bool jog_robot~%bool publish_traj ~%bool build_traj~%bool positioning_~%geometry_msgs/Twist p_gains_~%geometry_msgs/Twist d_gains_~%bool move_signal_~%bool tune_gain~%float64 amplitude~%int64 axis~%bool exit_~%float64 distance_to_contact_~%bool fake_distance_~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UI-request)))
  "Returns full string definition for message of type 'UI-request"
  (cl:format cl:nil "bool play_traj~%bool jog_robot~%bool publish_traj ~%bool build_traj~%bool positioning_~%geometry_msgs/Twist p_gains_~%geometry_msgs/Twist d_gains_~%bool move_signal_~%bool tune_gain~%float64 amplitude~%int64 axis~%bool exit_~%float64 distance_to_contact_~%bool fake_distance_~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UI-request>))
  (cl:+ 0
     1
     1
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p_gains_))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'd_gains_))
     1
     1
     8
     8
     1
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UI-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UI-request
    (cl:cons ':play_traj (play_traj msg))
    (cl:cons ':jog_robot (jog_robot msg))
    (cl:cons ':publish_traj (publish_traj msg))
    (cl:cons ':build_traj (build_traj msg))
    (cl:cons ':positioning_ (positioning_ msg))
    (cl:cons ':p_gains_ (p_gains_ msg))
    (cl:cons ':d_gains_ (d_gains_ msg))
    (cl:cons ':move_signal_ (move_signal_ msg))
    (cl:cons ':tune_gain (tune_gain msg))
    (cl:cons ':amplitude (amplitude msg))
    (cl:cons ':axis (axis msg))
    (cl:cons ':exit_ (exit_ msg))
    (cl:cons ':distance_to_contact_ (distance_to_contact_ msg))
    (cl:cons ':fake_distance_ (fake_distance_ msg))
))
;//! \htmlinclude UI-response.msg.html

(cl:defclass <UI-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UI-response (<UI-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UI-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UI-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_mpc-srv:<UI-response> is deprecated: use panda_mpc-srv:UI-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <UI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:result-val is deprecated.  Use panda_mpc-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UI-response>) ostream)
  "Serializes a message object of type '<UI-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UI-response>) istream)
  "Deserializes a message object of type '<UI-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UI-response>)))
  "Returns string type for a service object of type '<UI-response>"
  "panda_mpc/UIResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UI-response)))
  "Returns string type for a service object of type 'UI-response"
  "panda_mpc/UIResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UI-response>)))
  "Returns md5sum for a message object of type '<UI-response>"
  "2e1bdac3ef57deb56fae70ed4385f616")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UI-response)))
  "Returns md5sum for a message object of type 'UI-response"
  "2e1bdac3ef57deb56fae70ed4385f616")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UI-response>)))
  "Returns full string definition for message of type '<UI-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UI-response)))
  "Returns full string definition for message of type 'UI-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UI-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UI-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UI-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UI)))
  'UI-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UI)))
  'UI-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UI)))
  "Returns string type for a service object of type '<UI>"
  "panda_mpc/UI")