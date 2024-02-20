; Auto-generated. Do not edit!


(cl:in-package ekg_auv_testing-msg)


;//! \htmlinclude USBLResponseSim.msg.html

(cl:defclass <USBLResponseSim> (roslisp-msg-protocol:ros-message)
  ((transceiverID
    :reader transceiverID
    :initarg :transceiverID
    :type cl:integer
    :initform 0)
   (responseID
    :reader responseID
    :initarg :responseID
    :type cl:integer
    :initform 0)
   (transceiverModelName
    :reader transceiverModelName
    :initarg :transceiverModelName
    :type cl:string
    :initform "")
   (data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass USBLResponseSim (<USBLResponseSim>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <USBLResponseSim>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'USBLResponseSim)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ekg_auv_testing-msg:<USBLResponseSim> is deprecated: use ekg_auv_testing-msg:USBLResponseSim instead.")))

(cl:ensure-generic-function 'transceiverID-val :lambda-list '(m))
(cl:defmethod transceiverID-val ((m <USBLResponseSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ekg_auv_testing-msg:transceiverID-val is deprecated.  Use ekg_auv_testing-msg:transceiverID instead.")
  (transceiverID m))

(cl:ensure-generic-function 'responseID-val :lambda-list '(m))
(cl:defmethod responseID-val ((m <USBLResponseSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ekg_auv_testing-msg:responseID-val is deprecated.  Use ekg_auv_testing-msg:responseID instead.")
  (responseID m))

(cl:ensure-generic-function 'transceiverModelName-val :lambda-list '(m))
(cl:defmethod transceiverModelName-val ((m <USBLResponseSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ekg_auv_testing-msg:transceiverModelName-val is deprecated.  Use ekg_auv_testing-msg:transceiverModelName instead.")
  (transceiverModelName m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <USBLResponseSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ekg_auv_testing-msg:data-val is deprecated.  Use ekg_auv_testing-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <USBLResponseSim>) ostream)
  "Serializes a message object of type '<USBLResponseSim>"
  (cl:let* ((signed (cl:slot-value msg 'transceiverID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'responseID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'transceiverModelName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'transceiverModelName))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <USBLResponseSim>) istream)
  "Deserializes a message object of type '<USBLResponseSim>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'transceiverID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'responseID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'transceiverModelName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'transceiverModelName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<USBLResponseSim>)))
  "Returns string type for a message object of type '<USBLResponseSim>"
  "ekg_auv_testing/USBLResponseSim")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'USBLResponseSim)))
  "Returns string type for a message object of type 'USBLResponseSim"
  "ekg_auv_testing/USBLResponseSim")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<USBLResponseSim>)))
  "Returns md5sum for a message object of type '<USBLResponseSim>"
  "23b647a5a32e3bbb7ba751a1824c350f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'USBLResponseSim)))
  "Returns md5sum for a message object of type 'USBLResponseSim"
  "23b647a5a32e3bbb7ba751a1824c350f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<USBLResponseSim>)))
  "Returns full string definition for message of type '<USBLResponseSim>"
  (cl:format cl:nil "int32 transceiverID~%int32 responseID~%string transceiverModelName~%string data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'USBLResponseSim)))
  "Returns full string definition for message of type 'USBLResponseSim"
  (cl:format cl:nil "int32 transceiverID~%int32 responseID~%string transceiverModelName~%string data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <USBLResponseSim>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'transceiverModelName))
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <USBLResponseSim>))
  "Converts a ROS message object to a list"
  (cl:list 'USBLResponseSim
    (cl:cons ':transceiverID (transceiverID msg))
    (cl:cons ':responseID (responseID msg))
    (cl:cons ':transceiverModelName (transceiverModelName msg))
    (cl:cons ':data (data msg))
))
