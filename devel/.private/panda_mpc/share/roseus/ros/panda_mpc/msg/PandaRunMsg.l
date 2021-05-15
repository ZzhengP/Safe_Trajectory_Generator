;; Auto-generated. Do not edit!


(when (boundp 'panda_mpc::PandaRunMsg)
  (if (not (find-package "PANDA_MPC"))
    (make-package "PANDA_MPC"))
  (shadow 'PandaRunMsg (find-package "PANDA_MPC")))
(unless (find-package "PANDA_MPC::PANDARUNMSG")
  (make-package "PANDA_MPC::PANDARUNMSG"))

(in-package "ROS")
;;//! \htmlinclude PandaRunMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass panda_mpc::PandaRunMsg
  :super ros::object
  :slots (_header _vmax_ec _t_traj_curr _Xd_traj _Xdd_traj _X_err _Xd_control _qd_des _play_traj_ _positioning_ _tune_gains_ _distance_to_contact_ ))

(defmethod panda_mpc::PandaRunMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:vmax_ec __vmax_ec) 0.0)
    ((:t_traj_curr __t_traj_curr) 0.0)
    ((:Xd_traj __Xd_traj) (instance geometry_msgs::Twist :init))
    ((:Xdd_traj __Xdd_traj) (instance geometry_msgs::Twist :init))
    ((:X_err __X_err) (instance geometry_msgs::Twist :init))
    ((:Xd_control __Xd_control) (instance geometry_msgs::Twist :init))
    ((:qd_des __qd_des) (instance sensor_msgs::JointState :init))
    ((:play_traj_ __play_traj_) nil)
    ((:positioning_ __positioning_) nil)
    ((:tune_gains_ __tune_gains_) nil)
    ((:distance_to_contact_ __distance_to_contact_) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _vmax_ec (float __vmax_ec))
   (setq _t_traj_curr (float __t_traj_curr))
   (setq _Xd_traj __Xd_traj)
   (setq _Xdd_traj __Xdd_traj)
   (setq _X_err __X_err)
   (setq _Xd_control __Xd_control)
   (setq _qd_des __qd_des)
   (setq _play_traj_ __play_traj_)
   (setq _positioning_ __positioning_)
   (setq _tune_gains_ __tune_gains_)
   (setq _distance_to_contact_ (float __distance_to_contact_))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:vmax_ec
   (&optional __vmax_ec)
   (if __vmax_ec (setq _vmax_ec __vmax_ec)) _vmax_ec)
  (:t_traj_curr
   (&optional __t_traj_curr)
   (if __t_traj_curr (setq _t_traj_curr __t_traj_curr)) _t_traj_curr)
  (:Xd_traj
   (&rest __Xd_traj)
   (if (keywordp (car __Xd_traj))
       (send* _Xd_traj __Xd_traj)
     (progn
       (if __Xd_traj (setq _Xd_traj (car __Xd_traj)))
       _Xd_traj)))
  (:Xdd_traj
   (&rest __Xdd_traj)
   (if (keywordp (car __Xdd_traj))
       (send* _Xdd_traj __Xdd_traj)
     (progn
       (if __Xdd_traj (setq _Xdd_traj (car __Xdd_traj)))
       _Xdd_traj)))
  (:X_err
   (&rest __X_err)
   (if (keywordp (car __X_err))
       (send* _X_err __X_err)
     (progn
       (if __X_err (setq _X_err (car __X_err)))
       _X_err)))
  (:Xd_control
   (&rest __Xd_control)
   (if (keywordp (car __Xd_control))
       (send* _Xd_control __Xd_control)
     (progn
       (if __Xd_control (setq _Xd_control (car __Xd_control)))
       _Xd_control)))
  (:qd_des
   (&rest __qd_des)
   (if (keywordp (car __qd_des))
       (send* _qd_des __qd_des)
     (progn
       (if __qd_des (setq _qd_des (car __qd_des)))
       _qd_des)))
  (:play_traj_
   (&optional __play_traj_)
   (if __play_traj_ (setq _play_traj_ __play_traj_)) _play_traj_)
  (:positioning_
   (&optional __positioning_)
   (if __positioning_ (setq _positioning_ __positioning_)) _positioning_)
  (:tune_gains_
   (&optional __tune_gains_)
   (if __tune_gains_ (setq _tune_gains_ __tune_gains_)) _tune_gains_)
  (:distance_to_contact_
   (&optional __distance_to_contact_)
   (if __distance_to_contact_ (setq _distance_to_contact_ __distance_to_contact_)) _distance_to_contact_)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _vmax_ec
    8
    ;; float64 _t_traj_curr
    8
    ;; geometry_msgs/Twist _Xd_traj
    (send _Xd_traj :serialization-length)
    ;; geometry_msgs/Twist _Xdd_traj
    (send _Xdd_traj :serialization-length)
    ;; geometry_msgs/Twist _X_err
    (send _X_err :serialization-length)
    ;; geometry_msgs/Twist _Xd_control
    (send _Xd_control :serialization-length)
    ;; sensor_msgs/JointState _qd_des
    (send _qd_des :serialization-length)
    ;; bool _play_traj_
    1
    ;; bool _positioning_
    1
    ;; bool _tune_gains_
    1
    ;; float64 _distance_to_contact_
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _vmax_ec
       (sys::poke _vmax_ec (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t_traj_curr
       (sys::poke _t_traj_curr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Twist _Xd_traj
       (send _Xd_traj :serialize s)
     ;; geometry_msgs/Twist _Xdd_traj
       (send _Xdd_traj :serialize s)
     ;; geometry_msgs/Twist _X_err
       (send _X_err :serialize s)
     ;; geometry_msgs/Twist _Xd_control
       (send _Xd_control :serialize s)
     ;; sensor_msgs/JointState _qd_des
       (send _qd_des :serialize s)
     ;; bool _play_traj_
       (if _play_traj_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _positioning_
       (if _positioning_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _tune_gains_
       (if _tune_gains_ (write-byte -1 s) (write-byte 0 s))
     ;; float64 _distance_to_contact_
       (sys::poke _distance_to_contact_ (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _vmax_ec
     (setq _vmax_ec (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t_traj_curr
     (setq _t_traj_curr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Twist _Xd_traj
     (send _Xd_traj :deserialize buf ptr-) (incf ptr- (send _Xd_traj :serialization-length))
   ;; geometry_msgs/Twist _Xdd_traj
     (send _Xdd_traj :deserialize buf ptr-) (incf ptr- (send _Xdd_traj :serialization-length))
   ;; geometry_msgs/Twist _X_err
     (send _X_err :deserialize buf ptr-) (incf ptr- (send _X_err :serialization-length))
   ;; geometry_msgs/Twist _Xd_control
     (send _Xd_control :deserialize buf ptr-) (incf ptr- (send _Xd_control :serialization-length))
   ;; sensor_msgs/JointState _qd_des
     (send _qd_des :deserialize buf ptr-) (incf ptr- (send _qd_des :serialization-length))
   ;; bool _play_traj_
     (setq _play_traj_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _positioning_
     (setq _positioning_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _tune_gains_
     (setq _tune_gains_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _distance_to_contact_
     (setq _distance_to_contact_ (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get panda_mpc::PandaRunMsg :md5sum-) "978d838c2f0b7635c26f79d5649e9439")
(setf (get panda_mpc::PandaRunMsg :datatype-) "panda_mpc/PandaRunMsg")
(setf (get panda_mpc::PandaRunMsg :definition-)
      "Header header
float64 vmax_ec # [m/s]
float64 t_traj_curr
geometry_msgs/Twist Xd_traj
geometry_msgs/Twist Xdd_traj
geometry_msgs/Twist X_err
geometry_msgs/Twist Xd_control
sensor_msgs/JointState qd_des
bool play_traj_
bool positioning_
bool tune_gains_
float64 distance_to_contact_
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
float64 z
================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

")



(provide :panda_mpc/PandaRunMsg "978d838c2f0b7635c26f79d5649e9439")


