; Auto-generated. Do not edit!


(cl:in-package fw_wrapper-srv)


;//! \htmlinclude getcmd-request.msg.html

(cl:defclass <getcmd-request> (roslisp-msg-protocol:ros-message)
  ((command_type
    :reader command_type
    :initarg :command_type
    :type cl:string
    :initform "")
   (device_id
    :reader device_id
    :initarg :device_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass getcmd-request (<getcmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getcmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getcmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fw_wrapper-srv:<getcmd-request> is deprecated: use fw_wrapper-srv:getcmd-request instead.")))

(cl:ensure-generic-function 'command_type-val :lambda-list '(m))
(cl:defmethod command_type-val ((m <getcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fw_wrapper-srv:command_type-val is deprecated.  Use fw_wrapper-srv:command_type instead.")
  (command_type m))

(cl:ensure-generic-function 'device_id-val :lambda-list '(m))
(cl:defmethod device_id-val ((m <getcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fw_wrapper-srv:device_id-val is deprecated.  Use fw_wrapper-srv:device_id instead.")
  (device_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getcmd-request>) ostream)
  "Serializes a message object of type '<getcmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command_type))
  (cl:let* ((signed (cl:slot-value msg 'device_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getcmd-request>) istream)
  "Deserializes a message object of type '<getcmd-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'device_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getcmd-request>)))
  "Returns string type for a service object of type '<getcmd-request>"
  "fw_wrapper/getcmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getcmd-request)))
  "Returns string type for a service object of type 'getcmd-request"
  "fw_wrapper/getcmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getcmd-request>)))
  "Returns md5sum for a message object of type '<getcmd-request>"
  "801c3de2e2f2dc00117707158c3f9477")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getcmd-request)))
  "Returns md5sum for a message object of type 'getcmd-request"
  "801c3de2e2f2dc00117707158c3f9477")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getcmd-request>)))
  "Returns full string definition for message of type '<getcmd-request>"
  (cl:format cl:nil "string command_type~%int8 device_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getcmd-request)))
  "Returns full string definition for message of type 'getcmd-request"
  (cl:format cl:nil "string command_type~%int8 device_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getcmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command_type))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getcmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getcmd-request
    (cl:cons ':command_type (command_type msg))
    (cl:cons ':device_id (device_id msg))
))
;//! \htmlinclude getcmd-response.msg.html

(cl:defclass <getcmd-response> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type cl:fixnum
    :initform 0))
)

(cl:defclass getcmd-response (<getcmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getcmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getcmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fw_wrapper-srv:<getcmd-response> is deprecated: use fw_wrapper-srv:getcmd-response instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <getcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fw_wrapper-srv:val-val is deprecated.  Use fw_wrapper-srv:val instead.")
  (val m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getcmd-response>) ostream)
  "Serializes a message object of type '<getcmd-response>"
  (cl:let* ((signed (cl:slot-value msg 'val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getcmd-response>) istream)
  "Deserializes a message object of type '<getcmd-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'val) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getcmd-response>)))
  "Returns string type for a service object of type '<getcmd-response>"
  "fw_wrapper/getcmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getcmd-response)))
  "Returns string type for a service object of type 'getcmd-response"
  "fw_wrapper/getcmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getcmd-response>)))
  "Returns md5sum for a message object of type '<getcmd-response>"
  "801c3de2e2f2dc00117707158c3f9477")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getcmd-response)))
  "Returns md5sum for a message object of type 'getcmd-response"
  "801c3de2e2f2dc00117707158c3f9477")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getcmd-response>)))
  "Returns full string definition for message of type '<getcmd-response>"
  (cl:format cl:nil "int16 val~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getcmd-response)))
  "Returns full string definition for message of type 'getcmd-response"
  (cl:format cl:nil "int16 val~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getcmd-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getcmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getcmd-response
    (cl:cons ':val (val msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getcmd)))
  'getcmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getcmd)))
  'getcmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getcmd)))
  "Returns string type for a service object of type '<getcmd>"
  "fw_wrapper/getcmd")