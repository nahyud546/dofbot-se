#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__JointInfo() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__msg__JointInfo__init(msg: *mut JointInfo) -> bool;
    fn dofbot_interface__msg__JointInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointInfo>, size: usize) -> bool;
    fn dofbot_interface__msg__JointInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointInfo>);
    fn dofbot_interface__msg__JointInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<JointInfo>) -> bool;
}

// Corresponds to dofbot_interface__msg__JointInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub name: rosidl_runtime_rs::String,


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

}



impl Default for JointInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__msg__JointInfo__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__msg__JointInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__JointInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__JointInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__JointInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointInfo where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/msg/JointInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__JointInfo() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__PosInfo() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__msg__PosInfo__init(msg: *mut PosInfo) -> bool;
    fn dofbot_interface__msg__PosInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PosInfo>, size: usize) -> bool;
    fn dofbot_interface__msg__PosInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PosInfo>);
    fn dofbot_interface__msg__PosInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PosInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<PosInfo>) -> bool;
}

// Corresponds to dofbot_interface__msg__PosInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PosInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pos1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pos2: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pos3: f64,


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



impl Default for PosInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__msg__PosInfo__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__msg__PosInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PosInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__PosInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__PosInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__PosInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PosInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PosInfo where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/msg/PosInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__PosInfo() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__Position() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__msg__Position__init(msg: *mut Position) -> bool;
    fn dofbot_interface__msg__Position__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Position>, size: usize) -> bool;
    fn dofbot_interface__msg__Position__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Position>);
    fn dofbot_interface__msg__Position__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Position>, out_seq: *mut rosidl_runtime_rs::Sequence<Position>) -> bool;
}

// Corresponds to dofbot_interface__msg__Position
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Position {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub z: f32,

}



impl Default for Position {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__msg__Position__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__msg__Position__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Position {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__Position__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__Position__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__Position__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Position {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Position where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/msg/Position";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__Position() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__ArmJoint() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__msg__ArmJoint__init(msg: *mut ArmJoint) -> bool;
    fn dofbot_interface__msg__ArmJoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ArmJoint>, size: usize) -> bool;
    fn dofbot_interface__msg__ArmJoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ArmJoint>);
    fn dofbot_interface__msg__ArmJoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ArmJoint>, out_seq: *mut rosidl_runtime_rs::Sequence<ArmJoint>) -> bool;
}

// Corresponds to dofbot_interface__msg__ArmJoint
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ArmJoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub run_time: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angle: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints: rosidl_runtime_rs::Sequence<f32>,

}



impl Default for ArmJoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__msg__ArmJoint__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__msg__ArmJoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ArmJoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__ArmJoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__ArmJoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__ArmJoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ArmJoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ArmJoint where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/msg/ArmJoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__ArmJoint() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__ImageMsg() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__msg__ImageMsg__init(msg: *mut ImageMsg) -> bool;
    fn dofbot_interface__msg__ImageMsg__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ImageMsg>, size: usize) -> bool;
    fn dofbot_interface__msg__ImageMsg__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ImageMsg>);
    fn dofbot_interface__msg__ImageMsg__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ImageMsg>, out_seq: *mut rosidl_runtime_rs::Sequence<ImageMsg>) -> bool;
}

// Corresponds to dofbot_interface__msg__ImageMsg
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ImageMsg {

    // This member is not documented.
    #[allow(missing_docs)]
    pub height: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub width: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub channels: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub data: rosidl_runtime_rs::Sequence<u8>,

}



impl Default for ImageMsg {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__msg__ImageMsg__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__msg__ImageMsg__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ImageMsg {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__ImageMsg__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__ImageMsg__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__ImageMsg__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ImageMsg {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ImageMsg where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/msg/ImageMsg";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__ImageMsg() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__WidthInfo() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__msg__WidthInfo__init(msg: *mut WidthInfo) -> bool;
    fn dofbot_interface__msg__WidthInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<WidthInfo>, size: usize) -> bool;
    fn dofbot_interface__msg__WidthInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<WidthInfo>);
    fn dofbot_interface__msg__WidthInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<WidthInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<WidthInfo>) -> bool;
}

// Corresponds to dofbot_interface__msg__WidthInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WidthInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub l_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub l_y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub r_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub r_y: f32,

}



impl Default for WidthInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__msg__WidthInfo__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__msg__WidthInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for WidthInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__WidthInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__WidthInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__WidthInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for WidthInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for WidthInfo where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/msg/WidthInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__WidthInfo() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__AprilTagInfo() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__msg__AprilTagInfo__init(msg: *mut AprilTagInfo) -> bool;
    fn dofbot_interface__msg__AprilTagInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AprilTagInfo>, size: usize) -> bool;
    fn dofbot_interface__msg__AprilTagInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AprilTagInfo>);
    fn dofbot_interface__msg__AprilTagInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AprilTagInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<AprilTagInfo>) -> bool;
}

// Corresponds to dofbot_interface__msg__AprilTagInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AprilTagInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub z: f32,

}



impl Default for AprilTagInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__msg__AprilTagInfo__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__msg__AprilTagInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AprilTagInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__AprilTagInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__AprilTagInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__AprilTagInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AprilTagInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AprilTagInfo where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/msg/AprilTagInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__AprilTagInfo() }
  }
}


#[link(name = "dofbot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__Yolov11Detect() -> *const std::ffi::c_void;
}

#[link(name = "dofbot_interface__rosidl_generator_c")]
extern "C" {
    fn dofbot_interface__msg__Yolov11Detect__init(msg: *mut Yolov11Detect) -> bool;
    fn dofbot_interface__msg__Yolov11Detect__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Yolov11Detect>, size: usize) -> bool;
    fn dofbot_interface__msg__Yolov11Detect__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Yolov11Detect>);
    fn dofbot_interface__msg__Yolov11Detect__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Yolov11Detect>, out_seq: *mut rosidl_runtime_rs::Sequence<Yolov11Detect>) -> bool;
}

// Corresponds to dofbot_interface__msg__Yolov11Detect
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Yolov11Detect {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub centerx: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub centery: f32,

}



impl Default for Yolov11Detect {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dofbot_interface__msg__Yolov11Detect__init(&mut msg as *mut _) {
        panic!("Call to dofbot_interface__msg__Yolov11Detect__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Yolov11Detect {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__Yolov11Detect__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__Yolov11Detect__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dofbot_interface__msg__Yolov11Detect__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Yolov11Detect {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Yolov11Detect where Self: Sized {
  const TYPE_NAME: &'static str = "dofbot_interface/msg/Yolov11Detect";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dofbot_interface__msg__Yolov11Detect() }
  }
}


