; Auto-generated. Do not edit!


(cl:in-package panda_mpc-srv)


;//! \htmlinclude UpdateTrajectoryNextPoint-request.msg.html

(cl:defclass <UpdateTrajectoryNextPoint-request> (roslisp-msg-protocol:ros-message)
  ((next_point
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

(cl:defclass UpdateTrajectoryNextPoint-request (<UpdateTrajectoryNextPoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateTrajectoryNextPoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateTrajectoryNextPoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_mpc-srv:<UpdateTrajectoryNextPoint-request> is deprecated: use panda_mpc-srv:UpdateTrajectoryNextPoint-request instead.")))

(cl:ensure-generic-function 'next_point-val :lambda-list '(m))
(cl:defmethod next_point-val ((m <UpdateTrajectoryNextPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:next_point-val is deprecated.  Use panda_mpc-srv:next_point instead.")
  (next_point m))

(cl:ensure-generic-function 'next_vel-val :lambda-list '(m))
(cl:defmethod next_vel-val ((m <UpdateTrajectoryNextPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:next_vel-val is deprecated.  Use panda_mpc-srv:next_vel instead.")
  (next_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateTrajectoryNextPoint-request>) ostream)
  "Serializes a message object of type '<UpdateTrajectoryNextPoint-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'next_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'next_vel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateTrajectoryNextPoint-request>) istream)
  "Deserializes a message object of type '<UpdateTrajectoryNextPoint-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'next_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'next_vel) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateTrajectoryNextPoint-request>)))
  "Returns string type for a service object of type '<UpdateTrajectoryNextPoint-request>"
  "panda_mpc/UpdateTrajectoryNextPointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateTrajectoryNextPoint-request)))
  "Returns string type for a service object of type 'UpdateTrajectoryNextPoint-request"
  "panda_mpc/UpdateTrajectoryNextPointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateTrajectoryNextPoint-request>)))
  "Returns md5sum for a message object of type '<UpdateTrajectoryNextPoint-request>"
  "07d2a1fe5433c3b25d4df2bdf8fc4f1d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateTrajectoryNextPoint-request)))
  "Returns md5sum for a message object of type 'UpdateTrajectoryNextPoint-request"
  "07d2a1fe5433c3b25d4df2bdf8fc4f1d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateTrajectoryNextPoint-request>)))
  "Returns full string definition for message of type '<UpdateTrajectoryNextPoint-request>"
  (cl:format cl:nil "geometry_msgs/Vector3 next_point~%geometry_msgs/Twist next_vel~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateTrajectoryNextPoint-request)))
  "Returns full string definition for message of type 'UpdateTrajectoryNextPoint-request"
  (cl:format cl:nil "geometry_msgs/Vector3 next_point~%geometry_msgs/Twist next_vel~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateTrajectoryNextPoint-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'next_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'next_vel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateTrajectoryNextPoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateTrajectoryNextPoint-request
    (cl:cons ':next_point (next_point msg))
    (cl:cons ':next_vel (next_vel msg))
))
;//! \htmlinclude UpdateTrajectoryNextPoint-response.msg.html

(cl:defclass <UpdateTrajectoryNextPoint-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UpdateTrajectoryNextPoint-response (<UpdateTrajectoryNextPoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateTrajectoryNextPoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateTrajectoryNextPoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_mpc-srv:<UpdateTrajectoryNextPoint-response> is deprecated: use panda_mpc-srv:UpdateTrajectoryNextPoint-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <UpdateTrajectoryNextPoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_mpc-srv:success-val is deprecated.  Use panda_mpc-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateTrajectoryNextPoint-response>) ostream)
  "Serializes a message object of type '<UpdateTrajectoryNextPoint-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateTrajectoryNextPoint-response>) istream)
  "Deserializes a message object of type '<UpdateTrajectoryNextPoint-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateTrajectoryNextPoint-response>)))
  "Returns string type for a service object of type '<UpdateTrajectoryNextPoint-response>"
  "panda_mpc/UpdateTrajectoryNextPointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateTrajectoryNextPoint-response)))
  "Returns string type for a service object of type 'UpdateTrajectoryNextPoint-response"
  "panda_mpc/UpdateTrajectoryNextPointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateTrajectoryNextPoint-response>)))
  "Returns md5sum for a message object of type '<UpdateTrajectoryNextPoint-response>"
  "07d2a1fe5433c3b25d4df2bdf8fc4f1d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateTrajectoryNextPoint-response)))
  "Returns md5sum for a message object of type 'UpdateTrajectoryNextPoint-response"
  "07d2a1fe5433c3b25d4df2bdf8fc4f1d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateTrajectoryNextPoint-response>)))
  "Returns full string definition for message of type '<UpdateTrajectoryNextPoint-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateTrajectoryNextPoint-response)))
  "Returns full string definition for message of type 'UpdateTrajectoryNextPoint-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateTrajectoryNextPoint-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateTrajectoryNextPoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateTrajectoryNextPoint-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateTrajectoryNextPoint)))
  'UpdateTrajectoryNextPoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateTrajectoryNextPoint)))
  'UpdateTrajectoryNextPoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateTrajectoryNextPoint)))
  "Returns string type for a service object of type '<UpdateTrajectoryNextPoint>"
  "panda_mpc/UpdateTrajectoryNextPoint")