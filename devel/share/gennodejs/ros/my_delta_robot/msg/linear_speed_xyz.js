// Auto-generated. Do not edit!

// (in-package my_delta_robot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class linear_speed_xyz {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.xo = null;
      this.yo = null;
      this.zo = null;
      this.xf = null;
      this.yf = null;
      this.zf = null;
      this.vmax = null;
      this.amax = null;
      this.gripper = null;
    }
    else {
      if (initObj.hasOwnProperty('xo')) {
        this.xo = initObj.xo
      }
      else {
        this.xo = 0.0;
      }
      if (initObj.hasOwnProperty('yo')) {
        this.yo = initObj.yo
      }
      else {
        this.yo = 0.0;
      }
      if (initObj.hasOwnProperty('zo')) {
        this.zo = initObj.zo
      }
      else {
        this.zo = 0.0;
      }
      if (initObj.hasOwnProperty('xf')) {
        this.xf = initObj.xf
      }
      else {
        this.xf = 0.0;
      }
      if (initObj.hasOwnProperty('yf')) {
        this.yf = initObj.yf
      }
      else {
        this.yf = 0.0;
      }
      if (initObj.hasOwnProperty('zf')) {
        this.zf = initObj.zf
      }
      else {
        this.zf = 0.0;
      }
      if (initObj.hasOwnProperty('vmax')) {
        this.vmax = initObj.vmax
      }
      else {
        this.vmax = 0.0;
      }
      if (initObj.hasOwnProperty('amax')) {
        this.amax = initObj.amax
      }
      else {
        this.amax = 0.0;
      }
      if (initObj.hasOwnProperty('gripper')) {
        this.gripper = initObj.gripper
      }
      else {
        this.gripper = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type linear_speed_xyz
    // Serialize message field [xo]
    bufferOffset = _serializer.float32(obj.xo, buffer, bufferOffset);
    // Serialize message field [yo]
    bufferOffset = _serializer.float32(obj.yo, buffer, bufferOffset);
    // Serialize message field [zo]
    bufferOffset = _serializer.float32(obj.zo, buffer, bufferOffset);
    // Serialize message field [xf]
    bufferOffset = _serializer.float32(obj.xf, buffer, bufferOffset);
    // Serialize message field [yf]
    bufferOffset = _serializer.float32(obj.yf, buffer, bufferOffset);
    // Serialize message field [zf]
    bufferOffset = _serializer.float32(obj.zf, buffer, bufferOffset);
    // Serialize message field [vmax]
    bufferOffset = _serializer.float32(obj.vmax, buffer, bufferOffset);
    // Serialize message field [amax]
    bufferOffset = _serializer.float32(obj.amax, buffer, bufferOffset);
    // Serialize message field [gripper]
    bufferOffset = _serializer.int64(obj.gripper, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type linear_speed_xyz
    let len;
    let data = new linear_speed_xyz(null);
    // Deserialize message field [xo]
    data.xo = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yo]
    data.yo = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [zo]
    data.zo = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [xf]
    data.xf = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yf]
    data.yf = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [zf]
    data.zf = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vmax]
    data.vmax = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [amax]
    data.amax = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gripper]
    data.gripper = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_delta_robot/linear_speed_xyz';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8043196b03be66e3bcd2ba1e7c560f64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 xo
    float32 yo
    float32 zo
    float32 xf
    float32 yf
    float32 zf
    float32 vmax
    float32 amax
    int64 gripper
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new linear_speed_xyz(null);
    if (msg.xo !== undefined) {
      resolved.xo = msg.xo;
    }
    else {
      resolved.xo = 0.0
    }

    if (msg.yo !== undefined) {
      resolved.yo = msg.yo;
    }
    else {
      resolved.yo = 0.0
    }

    if (msg.zo !== undefined) {
      resolved.zo = msg.zo;
    }
    else {
      resolved.zo = 0.0
    }

    if (msg.xf !== undefined) {
      resolved.xf = msg.xf;
    }
    else {
      resolved.xf = 0.0
    }

    if (msg.yf !== undefined) {
      resolved.yf = msg.yf;
    }
    else {
      resolved.yf = 0.0
    }

    if (msg.zf !== undefined) {
      resolved.zf = msg.zf;
    }
    else {
      resolved.zf = 0.0
    }

    if (msg.vmax !== undefined) {
      resolved.vmax = msg.vmax;
    }
    else {
      resolved.vmax = 0.0
    }

    if (msg.amax !== undefined) {
      resolved.amax = msg.amax;
    }
    else {
      resolved.amax = 0.0
    }

    if (msg.gripper !== undefined) {
      resolved.gripper = msg.gripper;
    }
    else {
      resolved.gripper = 0
    }

    return resolved;
    }
};

module.exports = linear_speed_xyz;
