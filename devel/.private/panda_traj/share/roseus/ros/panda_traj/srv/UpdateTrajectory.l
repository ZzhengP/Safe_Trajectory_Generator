;; Auto-generated. Do not edit!


(when (boundp 'panda_traj::UpdateTrajectory)
  (if (not (find-package "PANDA_TRAJ"))
    (make-package "PANDA_TRAJ"))
  (shadow 'UpdateTrajectory (find-package "PANDA_TRAJ")))
(unless (find-package "PANDA_TRAJ::UPDATETRAJECTORY")
  (make-package "PANDA_TRAJ::UPDATETRAJECTORY"))
(unless (find-package "PANDA_TRAJ::UPDATETRAJECTORYREQUEST")
  (make-package "PANDA_TRAJ::UPDATETRAJECTORYREQUEST"))
(unless (find-package "PANDA_TRAJ::UPDATETRAJECTORYRESPONSE")
  (make-package "PANDA_TRAJ::UPDATETRAJECTORYRESPONSE"))

(in-package "ROS")





(defclass panda_traj::UpdateTrajectoryRequest
  :super ros::object
  :slots (_csv_traj_path _verbose ))

(defmethod panda_traj::UpdateTrajectoryRequest
  (:init
   (&key
    ((:csv_traj_path __csv_traj_path) "")
    ((:verbose __verbose) nil)
    )
   (send-super :init)
   (setq _csv_traj_path (string __csv_traj_path))
   (setq _verbose __verbose)
   self)
  (:csv_traj_path
   (&optional __csv_traj_path)
   (if __csv_traj_path (setq _csv_traj_path __csv_traj_path)) _csv_traj_path)
  (:verbose
   (&optional __verbose)
   (if __verbose (setq _verbose __verbose)) _verbose)
  (:serialization-length
   ()
   (+
    ;; string _csv_traj_path
    4 (length _csv_traj_path)
    ;; bool _verbose
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _csv_traj_path
       (write-long (length _csv_traj_path) s) (princ _csv_traj_path s)
     ;; bool _verbose
       (if _verbose (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _csv_traj_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _csv_traj_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _verbose
     (setq _verbose (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass panda_traj::UpdateTrajectoryResponse
  :super ros::object
  :slots (_success ))

(defmethod panda_traj::UpdateTrajectoryResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass panda_traj::UpdateTrajectory
  :super ros::object
  :slots ())

(setf (get panda_traj::UpdateTrajectory :md5sum-) "df273a5e598ca3c1de49e8c78524e55a")
(setf (get panda_traj::UpdateTrajectory :datatype-) "panda_traj/UpdateTrajectory")
(setf (get panda_traj::UpdateTrajectory :request) panda_traj::UpdateTrajectoryRequest)
(setf (get panda_traj::UpdateTrajectory :response) panda_traj::UpdateTrajectoryResponse)

(defmethod panda_traj::UpdateTrajectoryRequest
  (:response () (instance panda_traj::UpdateTrajectoryResponse :init)))

(setf (get panda_traj::UpdateTrajectoryRequest :md5sum-) "df273a5e598ca3c1de49e8c78524e55a")
(setf (get panda_traj::UpdateTrajectoryRequest :datatype-) "panda_traj/UpdateTrajectoryRequest")
(setf (get panda_traj::UpdateTrajectoryRequest :definition-)
      "string csv_traj_path
bool verbose
---
bool success
")

(setf (get panda_traj::UpdateTrajectoryResponse :md5sum-) "df273a5e598ca3c1de49e8c78524e55a")
(setf (get panda_traj::UpdateTrajectoryResponse :datatype-) "panda_traj/UpdateTrajectoryResponse")
(setf (get panda_traj::UpdateTrajectoryResponse :definition-)
      "string csv_traj_path
bool verbose
---
bool success
")



(provide :panda_traj/UpdateTrajectory "df273a5e598ca3c1de49e8c78524e55a")


