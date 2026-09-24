#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__Kinemarics_Request() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__srv__Kinemarics_Request__init(msg: *mut Kinemarics_Request) -> bool;
    fn dofbot_interface__srv__Kinemarics_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Kinemarics_Request>, size: usize) -> bool;
    fn dofbot_interface__srv__Kinemarics_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Kinemarics_Request>);
    fn dofbot_interface__srv__Kinemarics_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Kinemarics_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Kinemarics_Request>) -> bool;
}

// Corresponds to dofbot_interface__srv__Kinemarics_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Kinemarics_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub tar_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tar_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tar_z: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub roll: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint2: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint3: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint4: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint5: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint6: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kin_name: rosidl_runtime_rs::String,

}



impl Default for Kinemarics_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__srv__Kinemarics_Request__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__srv__Kinemarics_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Kinemarics_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__Kinemarics_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__Kinemarics_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__Kinemarics_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Kinemarics_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Kinemarics_Request where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/srv/Kinemarics_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__Kinemarics_Request() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__Kinemarics_Response() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__srv__Kinemarics_Response__init(msg: *mut Kinemarics_Response) -> bool;
    fn dofbot_interface__srv__Kinemarics_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Kinemarics_Response>, size: usize) -> bool;
    fn dofbot_interface__srv__Kinemarics_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Kinemarics_Response>);
    fn dofbot_interface__srv__Kinemarics_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Kinemarics_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Kinemarics_Response>) -> bool;
}

// Corresponds to dofbot_interface__srv__Kinemarics_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Kinemarics_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub joint1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint2: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint3: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint4: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint5: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint6: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub z: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub roll: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f64,

}



impl Default for Kinemarics_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__srv__Kinemarics_Response__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__srv__Kinemarics_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Kinemarics_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__Kinemarics_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__Kinemarics_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__Kinemarics_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Kinemarics_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Kinemarics_Response where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/srv/Kinemarics_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__Kinemarics_Response() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__CurJoint_Request() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__srv__CurJoint_Request__init(msg: *mut CurJoint_Request) -> bool;
    fn dofbot_interface__srv__CurJoint_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CurJoint_Request>, size: usize) -> bool;
    fn dofbot_interface__srv__CurJoint_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CurJoint_Request>);
    fn dofbot_interface__srv__CurJoint_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CurJoint_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<CurJoint_Request>) -> bool;
}

// Corresponds to dofbot_interface__srv__CurJoint_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CurJoint_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub srv_joints: rosidl_runtime_rs::String,

}



impl Default for CurJoint_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__srv__CurJoint_Request__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__srv__CurJoint_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CurJoint_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__CurJoint_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__CurJoint_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__CurJoint_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CurJoint_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CurJoint_Request where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/srv/CurJoint_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__CurJoint_Request() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__CurJoint_Response() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__srv__CurJoint_Response__init(msg: *mut CurJoint_Response) -> bool;
    fn dofbot_interface__srv__CurJoint_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CurJoint_Response>, size: usize) -> bool;
    fn dofbot_interface__srv__CurJoint_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CurJoint_Response>);
    fn dofbot_interface__srv__CurJoint_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CurJoint_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<CurJoint_Response>) -> bool;
}

// Corresponds to dofbot_interface__srv__CurJoint_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CurJoint_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub srv_joint1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub srv_joint2: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub srv_joint3: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub srv_joint4: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub srv_joint5: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub srv_joint6: f64,

}



impl Default for CurJoint_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__srv__CurJoint_Response__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__srv__CurJoint_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CurJoint_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__CurJoint_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__CurJoint_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__CurJoint_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CurJoint_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CurJoint_Response where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/srv/CurJoint_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__CurJoint_Response() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__DofbotProKinemarics_Request() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__srv__DofbotProKinemarics_Request__init(msg: *mut DofbotProKinemarics_Request) -> bool;
    fn dofbot_interface__srv__DofbotProKinemarics_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DofbotProKinemarics_Request>, size: usize) -> bool;
    fn dofbot_interface__srv__DofbotProKinemarics_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DofbotProKinemarics_Request>);
    fn dofbot_interface__srv__DofbotProKinemarics_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DofbotProKinemarics_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DofbotProKinemarics_Request>) -> bool;
}

// Corresponds to dofbot_interface__srv__DofbotProKinemarics_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DofbotProKinemarics_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub tar_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tar_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tar_z: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub roll: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint2: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint3: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint4: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint5: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cur_joint6: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kin_name: rosidl_runtime_rs::String,

}



impl Default for DofbotProKinemarics_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__srv__DofbotProKinemarics_Request__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__srv__DofbotProKinemarics_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DofbotProKinemarics_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__DofbotProKinemarics_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__DofbotProKinemarics_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__DofbotProKinemarics_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DofbotProKinemarics_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DofbotProKinemarics_Request where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/srv/DofbotProKinemarics_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__DofbotProKinemarics_Request() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__DofbotProKinemarics_Response() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__srv__DofbotProKinemarics_Response__init(msg: *mut DofbotProKinemarics_Response) -> bool;
    fn dofbot_interface__srv__DofbotProKinemarics_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DofbotProKinemarics_Response>, size: usize) -> bool;
    fn dofbot_interface__srv__DofbotProKinemarics_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DofbotProKinemarics_Response>);
    fn dofbot_interface__srv__DofbotProKinemarics_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DofbotProKinemarics_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DofbotProKinemarics_Response>) -> bool;
}

// Corresponds to dofbot_interface__srv__DofbotProKinemarics_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DofbotProKinemarics_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub joint1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint2: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint3: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint4: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint5: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint6: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub z: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub roll: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f64,

}



impl Default for DofbotProKinemarics_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__srv__DofbotProKinemarics_Response__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__srv__DofbotProKinemarics_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DofbotProKinemarics_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__DofbotProKinemarics_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__DofbotProKinemarics_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__srv__DofbotProKinemarics_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DofbotProKinemarics_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DofbotProKinemarics_Response where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/srv/DofbotProKinemarics_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__srv__DofbotProKinemarics_Response() }
  }
}






#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__dofbot_interface__srv__Kinemarics() -> *const std::ffi::c_void;
}

// Corresponds to dofbot_interface__srv__Kinemarics
#[allow(missing_docs, non_camel_case_types)]
pub struct Kinemarics;

impl rosidl_runtime_rs::Service for Kinemarics {
    type Request = Kinemarics_Request;
    type Response = Kinemarics_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__dofbot_interface__srv__Kinemarics() }
    }
}




#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__dofbot_interface__srv__CurJoint() -> *const std::ffi::c_void;
}

// Corresponds to dofbot_interface__srv__CurJoint
#[allow(missing_docs, non_camel_case_types)]
pub struct CurJoint;

impl rosidl_runtime_rs::Service for CurJoint {
    type Request = CurJoint_Request;
    type Response = CurJoint_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__dofbot_interface__srv__CurJoint() }
    }
}




#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__dofbot_interface__srv__DofbotProKinemarics() -> *const std::ffi::c_void;
}

// Corresponds to dofbot_interface__srv__DofbotProKinemarics
#[allow(missing_docs, non_camel_case_types)]
pub struct DofbotProKinemarics;

impl rosidl_runtime_rs::Service for DofbotProKinemarics {
    type Request = DofbotProKinemarics_Request;
    type Response = DofbotProKinemarics_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__dofbot_interface__srv__DofbotProKinemarics() }
    }
}


