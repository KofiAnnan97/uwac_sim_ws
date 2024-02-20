; Auto-generated. Do not edit!


(cl:in-package ekg_auv_testing-msg)


;//! \htmlinclude USBLRequestSim.msg.html

(cl:defclass <USBLRequestSim> (roslisp-msg-protocol:ros-message)
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
   (transponderModelName
    :reader transponderModelName
    :initarg :transponderModelName
    :type cl:string
    :initform "")
   (data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass USBLRequestSim (<USBLRequestSim>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <USBLRequestSim>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'USBLRequestSim)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ekg_auv_testing-msg:<USBLRequestSim> is deprecated: use ekg_auv_testing-msg:USBLRequestSim instead.")))

(cl:ensure-generic-function 'transceiverID-val :lambda-list '(m))
(cl:defmethod transceiverID-val ((m <USBLRequestSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ekg_auv_testing-msg:transceiverID-val is deprecated.  Use ekg_auv_testing-msg:transceiverID instead.")
  (transceiverID m))

(cl:ensure-generic-function 'responseID-val :lambda-list '(m))
(cl:defmethod responseID-val ((m <USBLRequestSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ekg_auv_testing-msg:responseID-val is deprecated.  Use ekg_auv_testing-msg:responseID instead.")
  (responseID m))

(cl:ensure-generic-function 'transponderModelName-val :lambda-list '(m))
(cl:defmethod transponderModelName-val ((m <USBLRequestSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ekg_auv_testing-msg:transponderModelName-val is deprecated.  Use ekg_auv_testing-msg:transponderModelName instead.")
  (transponderModelName m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <USBLRequestSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ekg_auv_testing-msg:data-val is deprecated.  Use ekg_auv_testing-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <USBLRequestSim>) ostream)
  "Serializes a message object of type '<USBLRequestSim>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'transponderModelName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'transponderModelName))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <USBLRequestSim>) istream)
  "Deserializes a message object of type '<USBLRequestSim>"
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
      (cl:setf (cl:slot-value msg 'transponderModelName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'transponderModelName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<USBLRequestSim>)))
  "Returns string type for a message object of type '<USBLRequestSim>"
  "ekg_auv_testing/USBLRequestSim")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'USBLRequestSim)))
  "Returns string type for a message object of type 'USBLRequestSim"
  "ekg_auv_testing/USBLRequestSim")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<USBLRequestSim>)))
  "Returns md5sum for a message object of type '<USBLRequestSim>"
  "4428d1e14a31634a968c9e0bbb5fc775")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'USBLRequestSim)))
  "Returns md5sum for a message object of type 'USBLRequestSim"
  "4428d1e14a31634a968c9e0bbb5fc775")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<USBLRequestSim>)))
  "Returns full string definition for message of type '<USBLRequestSim>"
  (cl:format cl:nil "int32 transceiverID~%int32 responseID~%string transponderModelName~%string data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'USBLRequestSim)))
  "Returns full string definition for message of type 'USBLRequestSim"
  (cl:format cl:nil "int32 transceiverID~%int32 responseID~%string transponderModelName~%string data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <USBLRequestSim>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'transponderModelName))
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <USBLRequestSim>))
  "Converts a ROS message object to a list"
  (cl:list 'USBLRequestSim
    (cl:cons ':transceiverID (transceiverID msg))
    (cl:cons ':responseID (responseID msg))
    (cl:cons ':transponderModelName (transponderModelName msg))
    (cl:cons ':data (data msg))
))
