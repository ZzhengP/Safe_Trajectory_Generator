;; Auto-generated. Do not edit!


(when (boundp 'panda_mpc::trajectoryAcceleration)
  (if (not (find-package "PANDA_MPC"))
    (make-package "PANDA_MPC"))
  (shadow 'trajectoryAcceleration (find-package "PANDA_MPC")))
(unless (find-package "PANDA_MPC::TRAJECTORYACCELERATION")
  (make-package "PANDA_MPC::TRAJECTORYACCELERATION"))

(in-package "ROS")
;;//! \htmlinclude trajectoryAcceleration.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass panda_mpc::trajectoryAcceleration
  :super ros::object
  :slots (_header _jntAcc ))

(defmethod panda_mpc::trajectoryAcceleration
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:jntAcc __jntAcc) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _jntAcc __jntAcc)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:jntAcc
   (&optional __jntAcc)
   (if __jntAcc (setq _jntAcc __jntAcc)) _jntAcc)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[] _jntAcc
    (* 4    (length _jntAcc)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[] _jntAcc
     (write-long (length _jntAcc) s)
     (dotimes (i (length _jntAcc))
       (sys::poke (elt _jntAcc i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[] _jntAcc
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _jntAcc (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _jntAcc i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get panda_mpc::trajectoryAcceleration :md5sum-) "afce3edbfaa1174f0c4a39872e9aa086")
(setf (get panda_mpc::trajectoryAcceleration :datatype-) "panda_mpc/trajectoryAcceleration")
(setf (get panda_mpc::trajectoryAcceleration :definition-)
      "Header header
float32[] jntAcc

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

")



(provide :panda_mpc/trajectoryAcceleration "afce3edbfaa1174f0c4a39872e9aa086")


