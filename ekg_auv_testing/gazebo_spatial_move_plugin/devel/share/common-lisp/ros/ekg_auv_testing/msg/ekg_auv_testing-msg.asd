
(cl:in-package :asdf)

(defsystem "ekg_auv_testing-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "USBLRequestSim" :depends-on ("_package_USBLRequestSim"))
    (:file "_package_USBLRequestSim" :depends-on ("_package"))
    (:file "USBLResponseSim" :depends-on ("_package_USBLResponseSim"))
    (:file "_package_USBLResponseSim" :depends-on ("_package"))
  ))