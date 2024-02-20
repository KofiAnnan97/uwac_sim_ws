;; Auto-generated. Do not edit!


(when (boundp 'ekg_auv_testing::USBLResponseSim)
  (if (not (find-package "EKG_AUV_TESTING"))
    (make-package "EKG_AUV_TESTING"))
  (shadow 'USBLResponseSim (find-package "EKG_AUV_TESTING")))
(unless (find-package "EKG_AUV_TESTING::USBLRESPONSESIM")
  (make-package "EKG_AUV_TESTING::USBLRESPONSESIM"))

(in-package "ROS")
;;//! \htmlinclude USBLResponseSim.msg.html


(defclass ekg_auv_testing::USBLResponseSim
  :super ros::object
  :slots (_transceiverID _responseID _transceiverModelName _data ))

(defmethod ekg_auv_testing::USBLResponseSim
  (:init
   (&key
    ((:transceiverID __transceiverID) 0)
    ((:responseID __responseID) 0)
    ((:transceiverModelName __transceiverModelName) "")
    ((:data __data) "")
    )
   (send-super :init)
   (setq _transceiverID (round __transceiverID))
   (setq _responseID (round __responseID))
   (setq _transceiverModelName (string __transceiverModelName))
   (setq _data (string __data))
   self)
  (:transceiverID
   (&optional __transceiverID)
   (if __transceiverID (setq _transceiverID __transceiverID)) _transceiverID)
  (:responseID
   (&optional __responseID)
   (if __responseID (setq _responseID __responseID)) _responseID)
  (:transceiverModelName
   (&optional __transceiverModelName)
   (if __transceiverModelName (setq _transceiverModelName __transceiverModelName)) _transceiverModelName)
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
    ;; string _transceiverModelName
    4 (length _transceiverModelName)
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
     ;; string _transceiverModelName
       (write-long (length _transceiverModelName) s) (princ _transceiverModelName s)
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
   ;; string _transceiverModelName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _transceiverModelName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ekg_auv_testing::USBLResponseSim :md5sum-) "23b647a5a32e3bbb7ba751a1824c350f")
(setf (get ekg_auv_testing::USBLResponseSim :datatype-) "ekg_auv_testing/USBLResponseSim")
(setf (get ekg_auv_testing::USBLResponseSim :definition-)
      "int32 transceiverID
int32 responseID
string transceiverModelName
string data
")



(provide :ekg_auv_testing/USBLResponseSim "23b647a5a32e3bbb7ba751a1824c350f")


