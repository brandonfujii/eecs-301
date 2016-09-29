; Auto-generated. Do not edit!


(cl:in-package fw_wrapper-srv)


;//! \htmlinclude allcmd-request.msg.html

(cl:defclass <allcmd-request> (roslisp-msg-protocol:ros-message)
  ((command_type
    :reader command_type
    :initarg :command_type
    :type cl:string
    :initform "")
   (device_id
    :reader device_id
    :initarg :device_id
    :type cl:fixnum
    :initform 0)
   (target_val
    :reader target_val
    :initarg :target_val
    :type cl:fixnum
    :initform 0)
   (n_dev
    :reader n_dev
    :initarg :n_dev
    :type cl:fixnum
    :initform 0)
   (dev_ids
    :reader dev_ids
    :initarg :dev_ids
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (target_vals
    :reader target_vals
    :initarg :target_vals
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass allcmd-request (<allcmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <allcmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'allcmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fw_wrapper-srv:<allcmd-request> is deprecated: use fw_wrapper-srv:allcmd-request instead.")))

(cl:ensure-generic-function 'command_type-val :lambda-list '(m))
(cl:defmethod command_type-val ((m <allcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fw_wrapper-srv:command_type-val is deprecated.  Use fw_wrapper-srv:command_type instead.")
  (command_type m))

(cl:ensure-generic-function 'device_id-val :lambda-list '(m))
(cl:defmethod device_id-val ((m <allcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fw_wrapper-srv:device_id-val is deprecated.  Use fw_wrapper-srv:device_id instead.")
  (device_id m))

(cl:ensure-generic-function 'target_val-val :lambda-list '(m))
(cl:defmethod target_val-val ((m <allcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fw_wrapper-srv:target_val-val is deprecated.  Use fw_wrapper-srv:target_val instead.")
  (target_val m))

(cl:ensure-generic-function 'n_dev-val :lambda-list '(m))
(cl:defmethod n_dev-val ((m <allcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fw_wrapper-srv:n_dev-val is deprecated.  Use fw_wrapper-srv:n_dev instead.")
  (n_dev m))

(cl:ensure-generic-function 'dev_ids-val :lambda-list '(m))
(cl:defmethod dev_ids-val ((m <allcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fw_wrapper-srv:dev_ids-val is deprecated.  Use fw_wrapper-srv:dev_ids instead.")
  (dev_ids m))

(cl:ensure-generic-function 'target_vals-val :lambda-list '(m))
(cl:defmethod target_vals-val ((m <allcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fw_wrapper-srv:target_vals-val is deprecated.  Use fw_wrapper-srv:target_vals instead.")
  (target_vals m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <allcmd-request>) ostream)
  "Serializes a message object of type '<allcmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command_type))
  (cl:let* ((signed (cl:slot-value msg 'device_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'target_val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'n_dev)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dev_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'dev_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'target_vals))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'target_vals))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <allcmd-request>) istream)
  "Deserializes a message object of type '<allcmd-request>"
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_val) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_dev) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dev_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dev_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'target_vals) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'target_vals)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<allcmd-request>)))
  "Returns string type for a service object of type '<allcmd-request>"
  "fw_wrapper/allcmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'allcmd-request)))
  "Returns string type for a service object of type 'allcmd-request"
  "fw_wrapper/allcmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<allcmd-request>)))
  "Returns md5sum for a message object of type '<allcmd-request>"
  "6c38294b540cc8b0902b3fe918c995c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'allcmd-request)))
  "Returns md5sum for a message object of type 'allcmd-request"
  "6c38294b540cc8b0902b3fe918c995c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<allcmd-request>)))
  "Returns full string definition for message of type '<allcmd-request>"
  (cl:format cl:nil "string command_type~%int8 device_id~%int16 target_val~%int8 n_dev~%int8[] dev_ids~%int16[] target_vals~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'allcmd-request)))
  "Returns full string definition for message of type 'allcmd-request"
  (cl:format cl:nil "string command_type~%int8 device_id~%int16 target_val~%int8 n_dev~%int8[] dev_ids~%int16[] target_vals~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <allcmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command_type))
     1
     2
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dev_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'target_vals) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <allcmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'allcmd-request
    (cl:cons ':command_type (command_type msg))
    (cl:cons ':device_id (device_id msg))
    (cl:cons ':target_val (target_val msg))
    (cl:cons ':n_dev (n_dev msg))
    (cl:cons ':dev_ids (dev_ids msg))
    (cl:cons ':target_vals (target_vals msg))
))
;//! \htmlinclude allcmd-response.msg.html

(cl:defclass <allcmd-response> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type cl:fixnum
    :initform 0))
)

(cl:defclass allcmd-response (<allcmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <allcmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'allcmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fw_wrapper-srv:<allcmd-response> is deprecated: use fw_wrapper-srv:allcmd-response instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <allcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fw_wrapper-srv:val-val is deprecated.  Use fw_wrapper-srv:val instead.")
  (val m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <allcmd-response>) ostream)
  "Serializes a message object of type '<allcmd-response>"
  (cl:let* ((signed (cl:slot-value msg 'val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <allcmd-response>) istream)
  "Deserializes a message object of type '<allcmd-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'val) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<allcmd-response>)))
  "Returns string type for a service object of type '<allcmd-response>"
  "fw_wrapper/allcmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'allcmd-response)))
  "Returns string type for a service object of type 'allcmd-response"
  "fw_wrapper/allcmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<allcmd-response>)))
  "Returns md5sum for a message object of type '<allcmd-response>"
  "6c38294b540cc8b0902b3fe918c995c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'allcmd-response)))
  "Returns md5sum for a message object of type 'allcmd-response"
  "6c38294b540cc8b0902b3fe918c995c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<allcmd-response>)))
  "Returns full string definition for message of type '<allcmd-response>"
  (cl:format cl:nil "int16 val~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'allcmd-response)))
  "Returns full string definition for message of type 'allcmd-response"
  (cl:format cl:nil "int16 val~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <allcmd-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <allcmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'allcmd-response
    (cl:cons ':val (val msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'allcmd)))
  'allcmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'allcmd)))
  'allcmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'allcmd)))
  "Returns string type for a service object of type '<allcmd>"
  "fw_wrapper/allcmd")