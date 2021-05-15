;; Auto-generated. Do not edit!


(when (boundp 'panda_traj::TrajProperties)
  (if (not (find-package "PANDA_TRAJ"))
    (make-package "PANDA_TRAJ"))
  (shadow 'TrajProperties (find-package "PANDA_TRAJ")))
(unless (find-package "PANDA_TRAJ::TRAJPROPERTIES")
  (make-package "PANDA_TRAJ::TRAJPROPERTIES"))

(in-package "ROS")
;;//! \htmlinclude TrajProperties.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass panda_traj::TrajProperties
  :super ros::object
  :slots (_play_traj_ _jogging_ _gain_tunning_ _move_ _index_ _amplitude _X_curr_ _X_des_jog_ ))

(defmethod panda_traj::TrajProperties
  (:init
   (&key
    ((:play_traj_ __play_traj_) nil)
    ((:jogging_ __jogging_) nil)
    ((:gain_tunning_ __gain_tunning_) nil)
    ((:move_ __move_) nil)
    ((:index_ __index_) 0)
    ((:amplitude __amplitude) 0.0)
    ((:X_curr_ __X_curr_) (instance geometry_msgs::Pose :init))
    ((:X_des_jog_ __X_des_jog_) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _play_traj_ __play_traj_)
   (setq _jogging_ __jogging_)
   (setq _gain_tunning_ __gain_tunning_)
   (setq _move_ __move_)
   (setq _index_ (round __index_))
   (setq _amplitude (float __amplitude))
   (setq _X_curr_ __X_curr_)
   (setq _X_des_jog_ __X_des_jog_)
   self)
  (:play_traj_
   (&optional __play_traj_)
   (if __play_traj_ (setq _play_traj_ __play_traj_)) _play_traj_)
  (:jogging_
   (&optional __jogging_)
   (if __jogging_ (setq _jogging_ __jogging_)) _jogging_)
  (:gain_tunning_
   (&optional __gain_tunning_)
   (if __gain_tunning_ (setq _gain_tunning_ __gain_tunning_)) _gain_tunning_)
  (:move_
   (&optional __move_)
   (if __move_ (setq _move_ __move_)) _move_)
  (:index_
   (&optional __index_)
   (if __index_ (setq _index_ __index_)) _index_)
  (:amplitude
   (&optional __amplitude)
   (if __amplitude (setq _amplitude __amplitude)) _amplitude)
  (:X_curr_
   (&rest __X_curr_)
   (if (keywordp (car __X_curr_))
       (send* _X_curr_ __X_curr_)
     (progn
       (if __X_curr_ (setq _X_curr_ (car __X_curr_)))
       _X_curr_)))
  (:X_des_jog_
   (&rest __X_des_jog_)
   (if (keywordp (car __X_des_jog_))
       (send* _X_des_jog_ __X_des_jog_)
     (progn
       (if __X_des_jog_ (setq _X_des_jog_ (car __X_des_jog_)))
       _X_des_jog_)))
  (:serialization-length
   ()
   (+
    ;; bool _play_traj_
    1
    ;; bool _jogging_
    1
    ;; bool _gain_tunning_
    1
    ;; bool _move_
    1
    ;; int64 _index_
    8
    ;; float64 _amplitude
    8
    ;; geometry_msgs/Pose _X_curr_
    (send _X_curr_ :serialization-length)
    ;; geometry_msgs/Pose _X_des_jog_
    (send _X_des_jog_ :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _play_traj_
       (if _play_traj_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _jogging_
       (if _jogging_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _gain_tunning_
       (if _gain_tunning_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _move_
       (if _move_ (write-byte -1 s) (write-byte 0 s))
     ;; int64 _index_
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _index_ (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _index_) (= (length (_index_ . bv)) 2)) ;; bignum
              (write-long (ash (elt (_index_ . bv) 0) 0) s)
              (write-long (ash (elt (_index_ . bv) 1) -1) s))
             ((and (class _index_) (= (length (_index_ . bv)) 1)) ;; big1
              (write-long (elt (_index_ . bv) 0) s)
              (write-long (if (>= _index_ 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _index_ s)(write-long (if (>= _index_ 0) 0 #xffffffff) s)))
     ;; float64 _amplitude
       (sys::poke _amplitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Pose _X_curr_
       (send _X_curr_ :serialize s)
     ;; geometry_msgs/Pose _X_des_jog_
       (send _X_des_jog_ :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _play_traj_
     (setq _play_traj_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _jogging_
     (setq _jogging_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gain_tunning_
     (setq _gain_tunning_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _move_
     (setq _move_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int64 _index_
#+(or :alpha :irix6 :x86_64)
      (setf _index_ (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _index_ (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _amplitude
     (setq _amplitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Pose _X_curr_
     (send _X_curr_ :deserialize buf ptr-) (incf ptr- (send _X_curr_ :serialization-length))
   ;; geometry_msgs/Pose _X_des_jog_
     (send _X_des_jog_ :deserialize buf ptr-) (incf ptr- (send _X_des_jog_ :serialization-length))
   ;;
   self)
  )

(setf (get panda_traj::TrajProperties :md5sum-) "8fb34236d88ea1e31629703f4e635b92")
(setf (get panda_traj::TrajProperties :datatype-) "panda_traj/TrajProperties")
(setf (get panda_traj::TrajProperties :definition-)
      "bool play_traj_
bool jogging_
bool gain_tunning_
bool move_
int64 index_
float64 amplitude
geometry_msgs/Pose X_curr_
geometry_msgs/Pose X_des_jog_
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

")



(provide :panda_traj/TrajProperties "8fb34236d88ea1e31629703f4e635b92")


