;; Auto-generated. Do not edit!


(when (boundp 'ekg_auv_testing::USBLRequestSim)
  (if (not (find-package "EKG_AUV_TESTING"))
    (make-package "EKG_AUV_TESTING"))
  (shadow 'USBLRequestSim (find-package "EKG_AUV_TESTING")))
(unless (find-package "EKG_AUV_TESTING::USBLREQUESTSIM")
  (make-package "EKG_AUV_TESTING::USBLREQUESTSIM"))

(in-package "ROS")
;;//! \htmlinclude USBLRequestSim.msg.html


(defclass ekg_auv_testing::USBLRequestSim
  :super ros::object
  :slots (_transceiverID _responseID _transponderModelName _data ))

(defmethod ekg_auv_testing::USBLRequestSim
  (:init
   (&key
    ((:transceiverID __transceiverID) 0)
    ((:responseID __responseID) 0)
    ((:transponderModelName __transponderModelName) "")
    ((:data __data) "")
    )
   (send-super :init)
   (setq _transceiverID (round __transceiverID))
   (setq _responseID (round __responseID))
   (setq _transponderModelName (string __transponderModelName))
   (setq _data (string __data))
   self)
  (:transceiverID
   (&optional __transceiverID)
   (if __transceiverID (setq _transceiverID __transceiverID)) _transceiverID)
  (:responseID
   (&optional __responseID)
   (if __responseID (setq _responseID __responseID)) _responseID)
  (:transponderModelName
   (&optional __transponderModelName)
   (if __transponderModelName (setq _transponderModelName __transponderModelName)) _transponderModelName)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32 _transceiverID
    4
    ;; int32 _responseID
    4
    ;; string _transponderModelName
    4 (length _transponderModelName)
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _transceiverID
       (write-long _transceiverID s)
     ;; int32 _responseID
       (write-long _responseID s)
     ;; string _transponderModelName
       (write-long (length _transponderModelName) s) (princ _transponderModelName s)
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _transceiverID
     (setq _transceiverID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _responseID
     (setq _responseID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _transponderModelName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _transponderModelName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ekg_auv_testing::USBLRequestSim :md5sum-) "4428d1e14a31634a968c9e0bbb5fc775")
(setf (get ekg_auv_testing::USBLRequestSim :datatype-) "ekg_auv_testing/USBLRequestSim")
(setf (get ekg_auv_testing::USBLRequestSim :definition-)
      "int32 transceiverID
int32 responseID
string transponderModelName
string data
")



(provide :ekg_auv_testing/USBLRequestSim "4428d1e14a31634a968c9e0bbb5fc775")


