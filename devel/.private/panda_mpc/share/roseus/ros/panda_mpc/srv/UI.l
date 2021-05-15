;; Auto-generated. Do not edit!


(when (boundp 'panda_mpc::UI)
  (if (not (find-package "PANDA_MPC"))
    (make-package "PANDA_MPC"))
  (shadow 'UI (find-package "PANDA_MPC")))
(unless (find-package "PANDA_MPC::UI")
  (make-package "PANDA_MPC::UI"))
(unless (find-package "PANDA_MPC::UIREQUEST")
  (make-package "PANDA_MPC::UIREQUEST"))
(unless (find-package "PANDA_MPC::UIRESPONSE")
  (make-package "PANDA_MPC::UIRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass panda_mpc::UIRequest
  :super ros::object
  :slots (_play_traj _jog_robot _publish_traj _build_traj _positioning_ _p_gains_ _d_gains_ _move_signal_ _tune_gain _amplitude _axis _exit_ _distance_to_contact_ _fake_distance_ ))

(defmethod panda_mpc::UIRequest
  (:init
   (&key
    ((:play_traj __play_traj) nil)
    ((:jog_robot __jog_robot) nil)
    ((:publish_traj __publish_traj) nil)
    ((:build_traj __build_traj) nil)
    ((:positioning_ __positioning_) nil)
    ((:p_gains_ __p_gains_) (instance geometry_msgs::Twist :init))
    ((:d_gains_ __d_gains_) (instance geometry_msgs::Twist :init))
    ((:move_signal_ __move_signal_) nil)
    ((:tune_gain __tune_gain) nil)
    ((:amplitude __amplitude) 0.0)
    ((:axis __axis) 0)
    ((:exit_ __exit_) nil)
    ((:distance_to_contact_ __distance_to_contact_) 0.0)
    ((:fake_distance_ __fake_distance_) nil)
    )
   (send-super :init)
   (setq _play_traj __play_traj)
   (setq _jog_robot __jog_robot)
   (setq _publish_traj __publish_traj)
   (setq _build_traj __build_traj)
   (setq _positioning_ __positioning_)
   (setq _p_gains_ __p_gains_)
   (setq _d_gains_ __d_gains_)
   (setq _move_signal_ __move_signal_)
   (setq _tune_gain __tune_gain)
   (setq _amplitude (float __amplitude))
   (setq _axis (round __axis))
   (setq _exit_ __exit_)
   (setq _distance_to_contact_ (float __distance_to_contact_))
   (setq _fake_distance_ __fake_distance_)
   self)
  (:play_traj
   (&optional __play_traj)
   (if __play_traj (setq _play_traj __play_traj)) _play_traj)
  (:jog_robot
   (&optional __jog_robot)
   (if __jog_robot (setq _jog_robot __jog_robot)) _jog_robot)
  (:publish_traj
   (&optional __publish_traj)
   (if __publish_traj (setq _publish_traj __publish_traj)) _publish_traj)
  (:build_traj
   (&optional __build_traj)
   (if __build_traj (setq _build_traj __build_traj)) _build_traj)
  (:positioning_
   (&optional __positioning_)
   (if __positioning_ (setq _positioning_ __positioning_)) _positioning_)
  (:p_gains_
   (&rest __p_gains_)
   (if (keywordp (car __p_gains_))
       (send* _p_gains_ __p_gains_)
     (progn
       (if __p_gains_ (setq _p_gains_ (car __p_gains_)))
       _p_gains_)))
  (:d_gains_
   (&rest __d_gains_)
   (if (keywordp (car __d_gains_))
       (send* _d_gains_ __d_gains_)
     (progn
       (if __d_gains_ (setq _d_gains_ (car __d_gains_)))
       _d_gains_)))
  (:move_signal_
   (&optional __move_signal_)
   (if __move_signal_ (setq _move_signal_ __move_signal_)) _move_signal_)
  (:tune_gain
   (&optional __tune_gain)
   (if __tune_gain (setq _tune_gain __tune_gain)) _tune_gain)
  (:amplitude
   (&optional __amplitude)
   (if __amplitude (setq _amplitude __amplitude)) _amplitude)
  (:axis
   (&optional __axis)
   (if __axis (setq _axis __axis)) _axis)
  (:exit_
   (&optional __exit_)
   (if __exit_ (setq _exit_ __exit_)) _exit_)
  (:distance_to_contact_
   (&optional __distance_to_contact_)
   (if __distance_to_contact_ (setq _distance_to_contact_ __distance_to_contact_)) _distance_to_contact_)
  (:fake_distance_
   (&optional __fake_distance_)
   (if __fake_distance_ (setq _fake_distance_ __fake_distance_)) _fake_distance_)
  (:serialization-length
   ()
   (+
    ;; bool _play_traj
    1
    ;; bool _jog_robot
    1
    ;; bool _publish_traj
    1
    ;; bool _build_traj
    1
    ;; bool _positioning_
    1
    ;; geometry_msgs/Twist _p_gains_
    (send _p_gains_ :serialization-length)
    ;; geometry_msgs/Twist _d_gains_
    (send _d_gains_ :serialization-length)
    ;; bool _move_signal_
    1
    ;; bool _tune_gain
    1
    ;; float64 _amplitude
    8
    ;; int64 _axis
    8
    ;; bool _exit_
    1
    ;; float64 _distance_to_contact_
    8
    ;; bool _fake_distance_
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _play_traj
       (if _play_traj (write-byte -1 s) (write-byte 0 s))
     ;; bool _jog_robot
       (if _jog_robot (write-byte -1 s) (write-byte 0 s))
     ;; bool _publish_traj
       (if _publish_traj (write-byte -1 s) (write-byte 0 s))
     ;; bool _build_traj
       (if _build_traj (write-byte -1 s) (write-byte 0 s))
     ;; bool _positioning_
       (if _positioning_ (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Twist _p_gains_
       (send _p_gains_ :serialize s)
     ;; geometry_msgs/Twist _d_gains_
       (send _d_gains_ :serialize s)
     ;; bool _move_signal_
       (if _move_signal_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _tune_gain
       (if _tune_gain (write-byte -1 s) (write-byte 0 s))
     ;; float64 _amplitude
       (sys::poke _amplitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _axis
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _axis (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _axis) (= (length (_axis . bv)) 2)) ;; bignum
              (write-long (ash (elt (_axis . bv) 0) 0) s)
              (write-long (ash (elt (_axis . bv) 1) -1) s))
             ((and (class _axis) (= (length (_axis . bv)) 1)) ;; big1
              (write-long (elt (_axis . bv) 0) s)
              (write-long (if (>= _axis 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _axis s)(write-long (if (>= _axis 0) 0 #xffffffff) s)))
     ;; bool _exit_
       (if _exit_ (write-byte -1 s) (write-byte 0 s))
     ;; float64 _distance_to_contact_
       (sys::poke _distance_to_contact_ (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _fake_distance_
       (if _fake_distance_ (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _play_traj
     (setq _play_traj (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _jog_robot
     (setq _jog_robot (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _publish_traj
     (setq _publish_traj (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _build_traj
     (setq _build_traj (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _positioning_
     (setq _positioning_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Twist _p_gains_
     (send _p_gains_ :deserialize buf ptr-) (incf ptr- (send _p_gains_ :serialization-length))
   ;; geometry_msgs/Twist _d_gains_
     (send _d_gains_ :deserialize buf ptr-) (incf ptr- (send _d_gains_ :serialization-length))
   ;; bool _move_signal_
     (setq _move_signal_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _tune_gain
     (setq _tune_gain (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _amplitude
     (setq _amplitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _axis
#+(or :alpha :irix6 :x86_64)
      (setf _axis (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _axis (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; bool _exit_
     (setq _exit_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _distance_to_contact_
     (setq _distance_to_contact_ (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _fake_distance_
     (setq _fake_distance_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass panda_mpc::UIResponse
  :super ros::object
  :slots (_result ))

(defmethod panda_mpc::UIResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass panda_mpc::UI
  :super ros::object
  :slots ())

(setf (get panda_mpc::UI :md5sum-) "2e1bdac3ef57deb56fae70ed4385f616")
(setf (get panda_mpc::UI :datatype-) "panda_mpc/UI")
(setf (get panda_mpc::UI :request) panda_mpc::UIRequest)
(setf (get panda_mpc::UI :response) panda_mpc::UIResponse)

(defmethod panda_mpc::UIRequest
  (:response () (instance panda_mpc::UIResponse :init)))

(setf (get panda_mpc::UIRequest :md5sum-) "2e1bdac3ef57deb56fae70ed4385f616")
(setf (get panda_mpc::UIRequest :datatype-) "panda_mpc/UIRequest")
(setf (get panda_mpc::UIRequest :definition-)
      "bool play_traj
bool jog_robot
bool publish_traj 
bool build_traj
bool positioning_
geometry_msgs/Twist p_gains_
geometry_msgs/Twist d_gains_
bool move_signal_
bool tune_gain
float64 amplitude
int64 axis
bool exit_
float64 distance_to_contact_
bool fake_distance_

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
---
bool result

")

(setf (get panda_mpc::UIResponse :md5sum-) "2e1bdac3ef57deb56fae70ed4385f616")
(setf (get panda_mpc::UIResponse :datatype-) "panda_mpc/UIResponse")
(setf (get panda_mpc::UIResponse :definition-)
      "bool play_traj
bool jog_robot
bool publish_traj 
bool build_traj
bool positioning_
geometry_msgs/Twist p_gains_
geometry_msgs/Twist d_gains_
bool move_signal_
bool tune_gain
float64 amplitude
int64 axis
bool exit_
float64 distance_to_contact_
bool fake_distance_

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
---
bool result

")



(provide :panda_mpc/UI "2e1bdac3ef57deb56fae70ed4385f616")


