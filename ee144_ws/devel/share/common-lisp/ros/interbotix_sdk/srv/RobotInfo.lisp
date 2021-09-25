; Auto-generated. Do not edit!


(cl:in-package interbotix_sdk-srv)


;//! \htmlinclude RobotInfo-request.msg.html

(cl:defclass <RobotInfo-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RobotInfo-request (<RobotInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interbotix_sdk-srv:<RobotInfo-request> is deprecated: use interbotix_sdk-srv:RobotInfo-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotInfo-request>) ostream)
  "Serializes a message object of type '<RobotInfo-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotInfo-request>) istream)
  "Deserializes a message object of type '<RobotInfo-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotInfo-request>)))
  "Returns string type for a service object of type '<RobotInfo-request>"
  "interbotix_sdk/RobotInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotInfo-request)))
  "Returns string type for a service object of type 'RobotInfo-request"
  "interbotix_sdk/RobotInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotInfo-request>)))
  "Returns md5sum for a message object of type '<RobotInfo-request>"
  "8cbf67af7802be3e000262518104df12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotInfo-request)))
  "Returns md5sum for a message object of type 'RobotInfo-request"
  "8cbf67af7802be3e000262518104df12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotInfo-request>)))
  "Returns full string definition for message of type '<RobotInfo-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotInfo-request)))
  "Returns full string definition for message of type 'RobotInfo-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotInfo-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotInfo-request
))
;//! \htmlinclude RobotInfo-response.msg.html

(cl:defclass <RobotInfo-response> (roslisp-msg-protocol:ros-message)
  ((joint_names
    :reader joint_names
    :initarg :joint_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (joint_ids
    :reader joint_ids
    :initarg :joint_ids
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (lower_joint_limits
    :reader lower_joint_limits
    :initarg :lower_joint_limits
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (upper_joint_limits
    :reader upper_joint_limits
    :initarg :upper_joint_limits
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (velocity_limits
    :reader velocity_limits
    :initarg :velocity_limits
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (lower_gripper_limit
    :reader lower_gripper_limit
    :initarg :lower_gripper_limit
    :type cl:float
    :initform 0.0)
   (upper_gripper_limit
    :reader upper_gripper_limit
    :initarg :upper_gripper_limit
    :type cl:float
    :initform 0.0)
   (use_gripper
    :reader use_gripper
    :initarg :use_gripper
    :type cl:boolean
    :initform cl:nil)
   (home_pos
    :reader home_pos
    :initarg :home_pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (sleep_pos
    :reader sleep_pos
    :initarg :sleep_pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (num_joints
    :reader num_joints
    :initarg :num_joints
    :type cl:fixnum
    :initform 0)
   (num_single_joints
    :reader num_single_joints
    :initarg :num_single_joints
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RobotInfo-response (<RobotInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interbotix_sdk-srv:<RobotInfo-response> is deprecated: use interbotix_sdk-srv:RobotInfo-response instead.")))

(cl:ensure-generic-function 'joint_names-val :lambda-list '(m))
(cl:defmethod joint_names-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:joint_names-val is deprecated.  Use interbotix_sdk-srv:joint_names instead.")
  (joint_names m))

(cl:ensure-generic-function 'joint_ids-val :lambda-list '(m))
(cl:defmethod joint_ids-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:joint_ids-val is deprecated.  Use interbotix_sdk-srv:joint_ids instead.")
  (joint_ids m))

(cl:ensure-generic-function 'lower_joint_limits-val :lambda-list '(m))
(cl:defmethod lower_joint_limits-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:lower_joint_limits-val is deprecated.  Use interbotix_sdk-srv:lower_joint_limits instead.")
  (lower_joint_limits m))

(cl:ensure-generic-function 'upper_joint_limits-val :lambda-list '(m))
(cl:defmethod upper_joint_limits-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:upper_joint_limits-val is deprecated.  Use interbotix_sdk-srv:upper_joint_limits instead.")
  (upper_joint_limits m))

(cl:ensure-generic-function 'velocity_limits-val :lambda-list '(m))
(cl:defmethod velocity_limits-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:velocity_limits-val is deprecated.  Use interbotix_sdk-srv:velocity_limits instead.")
  (velocity_limits m))

(cl:ensure-generic-function 'lower_gripper_limit-val :lambda-list '(m))
(cl:defmethod lower_gripper_limit-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:lower_gripper_limit-val is deprecated.  Use interbotix_sdk-srv:lower_gripper_limit instead.")
  (lower_gripper_limit m))

(cl:ensure-generic-function 'upper_gripper_limit-val :lambda-list '(m))
(cl:defmethod upper_gripper_limit-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:upper_gripper_limit-val is deprecated.  Use interbotix_sdk-srv:upper_gripper_limit instead.")
  (upper_gripper_limit m))

(cl:ensure-generic-function 'use_gripper-val :lambda-list '(m))
(cl:defmethod use_gripper-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:use_gripper-val is deprecated.  Use interbotix_sdk-srv:use_gripper instead.")
  (use_gripper m))

(cl:ensure-generic-function 'home_pos-val :lambda-list '(m))
(cl:defmethod home_pos-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:home_pos-val is deprecated.  Use interbotix_sdk-srv:home_pos instead.")
  (home_pos m))

(cl:ensure-generic-function 'sleep_pos-val :lambda-list '(m))
(cl:defmethod sleep_pos-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:sleep_pos-val is deprecated.  Use interbotix_sdk-srv:sleep_pos instead.")
  (sleep_pos m))

(cl:ensure-generic-function 'num_joints-val :lambda-list '(m))
(cl:defmethod num_joints-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:num_joints-val is deprecated.  Use interbotix_sdk-srv:num_joints instead.")
  (num_joints m))

(cl:ensure-generic-function 'num_single_joints-val :lambda-list '(m))
(cl:defmethod num_single_joints-val ((m <RobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_sdk-srv:num_single_joints-val is deprecated.  Use interbotix_sdk-srv:num_single_joints instead.")
  (num_single_joints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotInfo-response>) ostream)
  "Serializes a message object of type '<RobotInfo-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'joint_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'joint_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lower_joint_limits))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lower_joint_limits))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'upper_joint_limits))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'upper_joint_limits))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'velocity_limits))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'velocity_limits))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lower_gripper_limit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'upper_gripper_limit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_gripper) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'home_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'home_pos))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sleep_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'sleep_pos))
  (cl:let* ((signed (cl:slot-value msg 'num_joints)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_single_joints)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotInfo-response>) istream)
  "Deserializes a message object of type '<RobotInfo-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lower_joint_limits) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lower_joint_limits)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'upper_joint_limits) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'upper_joint_limits)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'velocity_limits) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'velocity_limits)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lower_gripper_limit) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'upper_gripper_limit) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'use_gripper) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'home_pos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'home_pos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sleep_pos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sleep_pos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_joints) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_single_joints) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotInfo-response>)))
  "Returns string type for a service object of type '<RobotInfo-response>"
  "interbotix_sdk/RobotInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotInfo-response)))
  "Returns string type for a service object of type 'RobotInfo-response"
  "interbotix_sdk/RobotInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotInfo-response>)))
  "Returns md5sum for a message object of type '<RobotInfo-response>"
  "8cbf67af7802be3e000262518104df12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotInfo-response)))
  "Returns md5sum for a message object of type 'RobotInfo-response"
  "8cbf67af7802be3e000262518104df12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotInfo-response>)))
  "Returns full string definition for message of type '<RobotInfo-response>"
  (cl:format cl:nil "~%string[] joint_names~%int16[] joint_ids~%float64[] lower_joint_limits~%float64[] upper_joint_limits~%float64[] velocity_limits~%float64 lower_gripper_limit~%float64 upper_gripper_limit~%bool use_gripper~%float64[] home_pos~%float64[] sleep_pos~%int8 num_joints~%int8 num_single_joints~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotInfo-response)))
  "Returns full string definition for message of type 'RobotInfo-response"
  (cl:format cl:nil "~%string[] joint_names~%int16[] joint_ids~%float64[] lower_joint_limits~%float64[] upper_joint_limits~%float64[] velocity_limits~%float64 lower_gripper_limit~%float64 upper_gripper_limit~%bool use_gripper~%float64[] home_pos~%float64[] sleep_pos~%int8 num_joints~%int8 num_single_joints~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotInfo-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lower_joint_limits) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'upper_joint_limits) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'velocity_limits) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     8
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'home_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sleep_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotInfo-response
    (cl:cons ':joint_names (joint_names msg))
    (cl:cons ':joint_ids (joint_ids msg))
    (cl:cons ':lower_joint_limits (lower_joint_limits msg))
    (cl:cons ':upper_joint_limits (upper_joint_limits msg))
    (cl:cons ':velocity_limits (velocity_limits msg))
    (cl:cons ':lower_gripper_limit (lower_gripper_limit msg))
    (cl:cons ':upper_gripper_limit (upper_gripper_limit msg))
    (cl:cons ':use_gripper (use_gripper msg))
    (cl:cons ':home_pos (home_pos msg))
    (cl:cons ':sleep_pos (sleep_pos msg))
    (cl:cons ':num_joints (num_joints msg))
    (cl:cons ':num_single_joints (num_single_joints msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotInfo)))
  'RobotInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotInfo)))
  'RobotInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotInfo)))
  "Returns string type for a service object of type '<RobotInfo>"
  "interbotix_sdk/RobotInfo")