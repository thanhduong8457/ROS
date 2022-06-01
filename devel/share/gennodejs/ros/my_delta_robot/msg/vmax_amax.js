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

class vmax_amax {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vmax = null;
      this.amax = null;
    }
    else {
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vmax_amax
    // Serialize message field [vmax]
    bufferOffset = _serializer.float64(obj.vmax, buffer, bufferOffset);
    // Serialize message field [amax]
    bufferOffset = _serializer.float64(obj.amax, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vmax_amax
    let len;
    let data = new vmax_amax(null);
    // Deserialize message field [vmax]
    data.vmax = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [amax]
    data.amax = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_delta_robot/vmax_amax';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ab9faecee53d06a238561f5c5a17a83';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 vmax
    float64 amax
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vmax_amax(null);
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

    return resolved;
    }
};

module.exports = vmax_amax;
