#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to dofbot_interface__msg__JointInfo

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub name: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::JointInfo::default())
  }
}

impl rosidl_runtime_rs::Message for JointInfo {
  type RmwMsg = super::msg::rmw::JointInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
        joint1: msg.joint1,
        joint2: msg.joint2,
        joint3: msg.joint3,
        joint4: msg.joint4,
        joint5: msg.joint5,
        joint6: msg.joint6,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
      joint1: msg.joint1,
      joint2: msg.joint2,
      joint3: msg.joint3,
      joint4: msg.joint4,
      joint5: msg.joint5,
      joint6: msg.joint6,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      name: msg.name.to_string(),
      joint1: msg.joint1,
      joint2: msg.joint2,
      joint3: msg.joint3,
      joint4: msg.joint4,
      joint5: msg.joint5,
      joint6: msg.joint6,
    }
  }
}


// Corresponds to dofbot_interface__msg__PosInfo

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PosInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub name: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PosInfo::default())
  }
}

impl rosidl_runtime_rs::Message for PosInfo {
  type RmwMsg = super::msg::rmw::PosInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
        pos1: msg.pos1,
        pos2: msg.pos2,
        pos3: msg.pos3,
        roll: msg.roll,
        pitch: msg.pitch,
        yaw: msg.yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
      pos1: msg.pos1,
      pos2: msg.pos2,
      pos3: msg.pos3,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      name: msg.name.to_string(),
      pos1: msg.pos1,
      pos2: msg.pos2,
      pos3: msg.pos3,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
    }
  }
}


// Corresponds to dofbot_interface__msg__Position

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Position::default())
  }
}

impl rosidl_runtime_rs::Message for Position {
  type RmwMsg = super::msg::rmw::Position;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        x: msg.x,
        y: msg.y,
        z: msg.z,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      x: msg.x,
      y: msg.y,
      z: msg.z,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      x: msg.x,
      y: msg.y,
      z: msg.z,
    }
  }
}


// Corresponds to dofbot_interface__msg__ArmJoint

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub joints: Vec<f32>,

}



impl Default for ArmJoint {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ArmJoint::default())
  }
}

impl rosidl_runtime_rs::Message for ArmJoint {
  type RmwMsg = super::msg::rmw::ArmJoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        run_time: msg.run_time,
        angle: msg.angle,
        joints: msg.joints.as_slice().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
      run_time: msg.run_time,
      angle: msg.angle,
        joints: msg.joints.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      run_time: msg.run_time,
      angle: msg.angle,
      joints: msg.joints.into(),
    }
  }
}


// Corresponds to dofbot_interface__msg__ImageMsg

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub data: Vec<u8>,

}



impl Default for ImageMsg {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ImageMsg::default())
  }
}

impl rosidl_runtime_rs::Message for ImageMsg {
  type RmwMsg = super::msg::rmw::ImageMsg;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        height: msg.height,
        width: msg.width,
        channels: msg.channels,
        data: msg.data.as_slice().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      height: msg.height,
      width: msg.width,
      channels: msg.channels,
        data: msg.data.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      height: msg.height,
      width: msg.width,
      channels: msg.channels,
      data: msg.data.into(),
    }
  }
}


// Corresponds to dofbot_interface__msg__WidthInfo

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::WidthInfo::default())
  }
}

impl rosidl_runtime_rs::Message for WidthInfo {
  type RmwMsg = super::msg::rmw::WidthInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        l_x: msg.l_x,
        l_y: msg.l_y,
        r_x: msg.r_x,
        r_y: msg.r_y,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      l_x: msg.l_x,
      l_y: msg.l_y,
      r_x: msg.r_x,
      r_y: msg.r_y,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      l_x: msg.l_x,
      l_y: msg.l_y,
      r_x: msg.r_x,
      r_y: msg.r_y,
    }
  }
}


// Corresponds to dofbot_interface__msg__AprilTagInfo

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AprilTagInfo::default())
  }
}

impl rosidl_runtime_rs::Message for AprilTagInfo {
  type RmwMsg = super::msg::rmw::AprilTagInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        x: msg.x,
        y: msg.y,
        z: msg.z,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
      x: msg.x,
      y: msg.y,
      z: msg.z,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      x: msg.x,
      y: msg.y,
      z: msg.z,
    }
  }
}


// Corresponds to dofbot_interface__msg__Yolov11Detect

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Yolov11Detect {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub centerx: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub centery: f32,

}



impl Default for Yolov11Detect {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Yolov11Detect::default())
  }
}

impl rosidl_runtime_rs::Message for Yolov11Detect {
  type RmwMsg = super::msg::rmw::Yolov11Detect;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: msg.result.as_str().into(),
        centerx: msg.centerx,
        centery: msg.centery,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: msg.result.as_str().into(),
      centerx: msg.centerx,
      centery: msg.centery,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      result: msg.result.to_string(),
      centerx: msg.centerx,
      centery: msg.centery,
    }
  }
}


