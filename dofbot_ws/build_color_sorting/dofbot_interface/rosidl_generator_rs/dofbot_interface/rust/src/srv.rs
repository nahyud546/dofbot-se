#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to dofbot_interface__srv__Kinemarics_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub kin_name: std::string::String,

}



impl Default for Kinemarics_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Kinemarics_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Kinemarics_Request {
  type RmwMsg = super::srv::rmw::Kinemarics_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        tar_x: msg.tar_x,
        tar_y: msg.tar_y,
        tar_z: msg.tar_z,
        roll: msg.roll,
        pitch: msg.pitch,
        yaw: msg.yaw,
        cur_joint1: msg.cur_joint1,
        cur_joint2: msg.cur_joint2,
        cur_joint3: msg.cur_joint3,
        cur_joint4: msg.cur_joint4,
        cur_joint5: msg.cur_joint5,
        cur_joint6: msg.cur_joint6,
        kin_name: msg.kin_name.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      tar_x: msg.tar_x,
      tar_y: msg.tar_y,
      tar_z: msg.tar_z,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      cur_joint1: msg.cur_joint1,
      cur_joint2: msg.cur_joint2,
      cur_joint3: msg.cur_joint3,
      cur_joint4: msg.cur_joint4,
      cur_joint5: msg.cur_joint5,
      cur_joint6: msg.cur_joint6,
        kin_name: msg.kin_name.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      tar_x: msg.tar_x,
      tar_y: msg.tar_y,
      tar_z: msg.tar_z,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      cur_joint1: msg.cur_joint1,
      cur_joint2: msg.cur_joint2,
      cur_joint3: msg.cur_joint3,
      cur_joint4: msg.cur_joint4,
      cur_joint5: msg.cur_joint5,
      cur_joint6: msg.cur_joint6,
      kin_name: msg.kin_name.to_string(),
    }
  }
}


// Corresponds to dofbot_interface__srv__Kinemarics_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Kinemarics_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Kinemarics_Response {
  type RmwMsg = super::srv::rmw::Kinemarics_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        joint1: msg.joint1,
        joint2: msg.joint2,
        joint3: msg.joint3,
        joint4: msg.joint4,
        joint5: msg.joint5,
        joint6: msg.joint6,
        x: msg.x,
        y: msg.y,
        z: msg.z,
        roll: msg.roll,
        pitch: msg.pitch,
        yaw: msg.yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      joint1: msg.joint1,
      joint2: msg.joint2,
      joint3: msg.joint3,
      joint4: msg.joint4,
      joint5: msg.joint5,
      joint6: msg.joint6,
      x: msg.x,
      y: msg.y,
      z: msg.z,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      joint1: msg.joint1,
      joint2: msg.joint2,
      joint3: msg.joint3,
      joint4: msg.joint4,
      joint5: msg.joint5,
      joint6: msg.joint6,
      x: msg.x,
      y: msg.y,
      z: msg.z,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
    }
  }
}


// Corresponds to dofbot_interface__srv__CurJoint_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CurJoint_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub srv_joints: std::string::String,

}



impl Default for CurJoint_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::CurJoint_Request::default())
  }
}

impl rosidl_runtime_rs::Message for CurJoint_Request {
  type RmwMsg = super::srv::rmw::CurJoint_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        srv_joints: msg.srv_joints.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        srv_joints: msg.srv_joints.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      srv_joints: msg.srv_joints.to_string(),
    }
  }
}


// Corresponds to dofbot_interface__srv__CurJoint_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::CurJoint_Response::default())
  }
}

impl rosidl_runtime_rs::Message for CurJoint_Response {
  type RmwMsg = super::srv::rmw::CurJoint_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        srv_joint1: msg.srv_joint1,
        srv_joint2: msg.srv_joint2,
        srv_joint3: msg.srv_joint3,
        srv_joint4: msg.srv_joint4,
        srv_joint5: msg.srv_joint5,
        srv_joint6: msg.srv_joint6,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      srv_joint1: msg.srv_joint1,
      srv_joint2: msg.srv_joint2,
      srv_joint3: msg.srv_joint3,
      srv_joint4: msg.srv_joint4,
      srv_joint5: msg.srv_joint5,
      srv_joint6: msg.srv_joint6,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      srv_joint1: msg.srv_joint1,
      srv_joint2: msg.srv_joint2,
      srv_joint3: msg.srv_joint3,
      srv_joint4: msg.srv_joint4,
      srv_joint5: msg.srv_joint5,
      srv_joint6: msg.srv_joint6,
    }
  }
}


// Corresponds to dofbot_interface__srv__DofbotProKinemarics_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub kin_name: std::string::String,

}



impl Default for DofbotProKinemarics_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::DofbotProKinemarics_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DofbotProKinemarics_Request {
  type RmwMsg = super::srv::rmw::DofbotProKinemarics_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        tar_x: msg.tar_x,
        tar_y: msg.tar_y,
        tar_z: msg.tar_z,
        roll: msg.roll,
        pitch: msg.pitch,
        yaw: msg.yaw,
        cur_joint1: msg.cur_joint1,
        cur_joint2: msg.cur_joint2,
        cur_joint3: msg.cur_joint3,
        cur_joint4: msg.cur_joint4,
        cur_joint5: msg.cur_joint5,
        cur_joint6: msg.cur_joint6,
        kin_name: msg.kin_name.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      tar_x: msg.tar_x,
      tar_y: msg.tar_y,
      tar_z: msg.tar_z,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      cur_joint1: msg.cur_joint1,
      cur_joint2: msg.cur_joint2,
      cur_joint3: msg.cur_joint3,
      cur_joint4: msg.cur_joint4,
      cur_joint5: msg.cur_joint5,
      cur_joint6: msg.cur_joint6,
        kin_name: msg.kin_name.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      tar_x: msg.tar_x,
      tar_y: msg.tar_y,
      tar_z: msg.tar_z,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      cur_joint1: msg.cur_joint1,
      cur_joint2: msg.cur_joint2,
      cur_joint3: msg.cur_joint3,
      cur_joint4: msg.cur_joint4,
      cur_joint5: msg.cur_joint5,
      cur_joint6: msg.cur_joint6,
      kin_name: msg.kin_name.to_string(),
    }
  }
}


// Corresponds to dofbot_interface__srv__DofbotProKinemarics_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::DofbotProKinemarics_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DofbotProKinemarics_Response {
  type RmwMsg = super::srv::rmw::DofbotProKinemarics_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        joint1: msg.joint1,
        joint2: msg.joint2,
        joint3: msg.joint3,
        joint4: msg.joint4,
        joint5: msg.joint5,
        joint6: msg.joint6,
        x: msg.x,
        y: msg.y,
        z: msg.z,
        roll: msg.roll,
        pitch: msg.pitch,
        yaw: msg.yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      joint1: msg.joint1,
      joint2: msg.joint2,
      joint3: msg.joint3,
      joint4: msg.joint4,
      joint5: msg.joint5,
      joint6: msg.joint6,
      x: msg.x,
      y: msg.y,
      z: msg.z,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      joint1: msg.joint1,
      joint2: msg.joint2,
      joint3: msg.joint3,
      joint4: msg.joint4,
      joint5: msg.joint5,
      joint6: msg.joint6,
      x: msg.x,
      y: msg.y,
      z: msg.z,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
    }
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


