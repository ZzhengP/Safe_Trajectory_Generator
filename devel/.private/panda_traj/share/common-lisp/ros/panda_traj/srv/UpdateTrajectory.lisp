; Auto-generated. Do not edit!


(cl:in-package panda_traj-srv)


;//! \htmlinclude UpdateTrajectory-request.msg.html

(cl:defclass <UpdateTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((csv_traj_path
    :reader csv_traj_path
    :initarg :csv_traj_path
    :type cl:string
    :initform "")
   (verbose
    :reader verbose
    :initarg :verbose
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UpdateTrajectory-request (<UpdateTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_traj-srv:<UpdateTrajectory-request> is deprecated: use panda_traj-srv:UpdateTrajectory-request instead.")))

(cl:ensure-generic-function 'csv_traj_path-val :lambda-list '(m))
(cl:defmethod csv_traj_path-val ((m <UpdateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-srv:csv_traj_path-val is deprecated.  Use panda_traj-srv:csv_traj_path instead.")
  (csv_traj_path m))

(cl:ensure-generic-function 'verbose-val :lambda-list '(m))
(cl:defmethod verbose-val ((m <UpdateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-srv:verbose-val is deprecated.  Use panda_traj-srv:verbose instead.")
  (verbose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateTrajectory-request>) ostream)
  "Serializes a message object of type '<UpdateTrajectory-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'csv_traj_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'csv_traj_path))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'verbose) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateTrajectory-request>) istream)
  "Deserializes a message object of type '<UpdateTrajectory-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'csv_traj_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'csv_traj_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'verbose) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateTrajectory-request>)))
  "Returns string type for a service object of type '<UpdateTrajectory-request>"
  "panda_traj/UpdateTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateTrajectory-request)))
  "Returns string type for a service object of type 'UpdateTrajectory-request"
  "panda_traj/UpdateTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateTrajectory-request>)))
  "Returns md5sum for a message object of type '<UpdateTrajectory-request>"
  "df273a5e598ca3c1de49e8c78524e55a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateTrajectory-request)))
  "Returns md5sum for a message object of type 'UpdateTrajectory-request"
  "df273a5e598ca3c1de49e8c78524e55a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateTrajectory-request>)))
  "Returns full string definition for message of type '<UpdateTrajectory-request>"
  (cl:format cl:nil "string csv_traj_path~%bool verbose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateTrajectory-request)))
  "Returns full string definition for message of type 'UpdateTrajectory-request"
  (cl:format cl:nil "string csv_traj_path~%bool verbose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateTrajectory-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'csv_traj_path))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateTrajectory-request
    (cl:cons ':csv_traj_path (csv_traj_path msg))
    (cl:cons ':verbose (verbose msg))
))
;//! \htmlinclude UpdateTrajectory-response.msg.html

(cl:defclass <UpdateTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UpdateTrajectory-response (<UpdateTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_traj-srv:<UpdateTrajectory-response> is deprecated: use panda_traj-srv:UpdateTrajectory-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <UpdateTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_traj-srv:success-val is deprecated.  Use panda_traj-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateTrajectory-response>) ostream)
  "Serializes a message object of type '<UpdateTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateTrajectory-response>) istream)
  "Deserializes a message object of type '<UpdateTrajectory-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateTrajectory-response>)))
  "Returns string type for a service object of type '<UpdateTrajectory-response>"
  "panda_traj/UpdateTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateTrajectory-response)))
  "Returns string type for a service object of type 'UpdateTrajectory-response"
  "panda_traj/UpdateTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateTrajectory-response>)))
  "Returns md5sum for a message object of type '<UpdateTrajectory-response>"
  "df273a5e598ca3c1de49e8c78524e55a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateTrajectory-response)))
  "Returns md5sum for a message object of type 'UpdateTrajectory-response"
  "df273a5e598ca3c1de49e8c78524e55a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateTrajectory-response>)))
  "Returns full string definition for message of type '<UpdateTrajectory-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateTrajectory-response)))
  "Returns full string definition for message of type 'UpdateTrajectory-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateTrajectory-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateTrajectory-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateTrajectory)))
  'UpdateTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateTrajectory)))
  'UpdateTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateTrajectory)))
  "Returns string type for a service object of type '<UpdateTrajectory>"
  "panda_traj/UpdateTrajectory")