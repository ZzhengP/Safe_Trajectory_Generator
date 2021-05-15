; Auto-generated. Do not edit!


(cl:in-package panda_traj-msg)


;//! \htmlinclude TrajProperties.msg.html

(cl:defclass <TrajProperties> (roslisp-msg-protocol:ros-message)
  ((play_traj_
    :reader play_traj_
    :initarg :play_traj_
    :type cl:boolean
    :initform cl:nil)
   (jogging_
    :reader jogging_
    :initarg :jogging_
    :type cl:boolean
    :initform cl:nil)
   (gain_tunning_
    :reader gain_tunning_
    :initarg :gain_tunning_
    :type cl:boolean
    :initform cl:nil)
   (move_
    :reader move_
    :initarg :move_
    :type cl:boolean
    :initform cl:nil)
   (index_
    :reader index_
    :initarg :index_
    :type cl:integer
    :initform 0)
   (amplitude
    :reader amplitude
    :initarg :amplitude
    :type cl:float
    :initform 0.0)
   (X_curr_
    :reader X_curr_
    :initarg :X_curr_
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (X_des_jog_
    :reader X_des_jog_
    :initarg :X_des_jog_
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass TrajProperties (<TrajProperties>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajProperties>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajProperties)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_traj-msg:<TrajProperties> is deprecated: use panda_traj-msg:TrajProperties instead.")))

(cl:ensure-generic-function 'play_traj_-val :lambda-list '(m))
(cl:defmethod play_traj_-val ((m <TrajProperties>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-msg:play_traj_-val is deprecated.  Use panda_traj-msg:play_traj_ instead.")
  (play_traj_ m))

(cl:ensure-generic-function 'jogging_-val :lambda-list '(m))
(cl:defmethod jogging_-val ((m <TrajProperties>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-msg:jogging_-val is deprecated.  Use panda_traj-msg:jogging_ instead.")
  (jogging_ m))

(cl:ensure-generic-function 'gain_tunning_-val :lambda-list '(m))
(cl:defmethod gain_tunning_-val ((m <TrajProperties>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-msg:gain_tunning_-val is deprecated.  Use panda_traj-msg:gain_tunning_ instead.")
  (gain_tunning_ m))

(cl:ensure-generic-function 'move_-val :lambda-list '(m))
(cl:defmethod move_-val ((m <TrajProperties>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-msg:move_-val is deprecated.  Use panda_traj-msg:move_ instead.")
  (move_ m))

(cl:ensure-generic-function 'index_-val :lambda-list '(m))
(cl:defmethod index_-val ((m <TrajProperties>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-msg:index_-val is deprecated.  Use panda_traj-msg:index_ instead.")
  (index_ m))

(cl:ensure-generic-function 'amplitude-val :lambda-list '(m))
(cl:defmethod amplitude-val ((m <TrajProperties>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-msg:amplitude-val is deprecated.  Use panda_traj-msg:amplitude instead.")
  (amplitude m))

(cl:ensure-generic-function 'X_curr_-val :lambda-list '(m))
(cl:defmethod X_curr_-val ((m <TrajProperties>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-msg:X_curr_-val is deprecated.  Use panda_traj-msg:X_curr_ instead.")
  (X_curr_ m))

(cl:ensure-generic-function 'X_des_jog_-val :lambda-list '(m))
(cl:defmethod X_des_jog_-val ((m <TrajProperties>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-msg:X_des_jog_-val is deprecated.  Use panda_traj-msg:X_des_jog_ instead.")
  (X_des_jog_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajProperties>) ostream)
  "Serializes a message object of type '<TrajProperties>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'play_traj_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'jogging_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gain_tunning_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'move_) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'index_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'amplitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'X_curr_) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'X_des_jog_) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajProperties>) istream)
  "Deserializes a message object of type '<TrajProperties>"
    (cl:setf (cl:slot-value msg 'play_traj_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'jogging_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'gain_tunning_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'move_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index_) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'X_curr_) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'X_des_jog_) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajProperties>)))
  "Returns string type for a message object of type '<TrajProperties>"
  "panda_traj/TrajProperties")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajProperties)))
  "Returns string type for a message object of type 'TrajProperties"
  "panda_traj/TrajProperties")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajProperties>)))
  "Returns md5sum for a message object of type '<TrajProperties>"
  "8fb34236d88ea1e31629703f4e635b92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajProperties)))
  "Returns md5sum for a message object of type 'TrajProperties"
  "8fb34236d88ea1e31629703f4e635b92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajProperties>)))
  "Returns full string definition for message of type '<TrajProperties>"
  (cl:format cl:nil "bool play_traj_~%bool jogging_~%bool gain_tunning_~%bool move_~%int64 index_~%float64 amplitude~%geometry_msgs/Pose X_curr_~%geometry_msgs/Pose X_des_jog_~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajProperties)))
  "Returns full string definition for message of type 'TrajProperties"
  (cl:format cl:nil "bool play_traj_~%bool jogging_~%bool gain_tunning_~%bool move_~%int64 index_~%float64 amplitude~%geometry_msgs/Pose X_curr_~%geometry_msgs/Pose X_des_jog_~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajProperties>))
  (cl:+ 0
     1
     1
     1
     1
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'X_curr_))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'X_des_jog_))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajProperties>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajProperties
    (cl:cons ':play_traj_ (play_traj_ msg))
    (cl:cons ':jogging_ (jogging_ msg))
    (cl:cons ':gain_tunning_ (gain_tunning_ msg))
    (cl:cons ':move_ (move_ msg))
    (cl:cons ':index_ (index_ msg))
    (cl:cons ':amplitude (amplitude msg))
    (cl:cons ':X_curr_ (X_curr_ msg))
    (cl:cons ':X_des_jog_ (X_des_jog_ msg))
))
