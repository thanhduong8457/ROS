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

class num_point {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_point_1 = null;
      this.num_point_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('num_point_1')) {
        this.num_point_1 = initObj.num_point_1
      }
      else {
        this.num_point_1 = 0;
      }
      if (initObj.hasOwnProperty('num_point_2')) {
        this.num_point_2 = initObj.num_point_2
      }
      else {
        this.num_point_2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type num_point
    // Serialize message field [num_point_1]
    bufferOffset = _serializer.int64(obj.num_point_1, buffer, bufferOffset);
    // Serialize message field [num_point_2]
    bufferOffset = _serializer.int64(obj.num_point_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type num_point
    let len;
    let data = new num_point(null);
    // Deserialize message field [num_point_1]
    data.num_point_1 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [num_point_2]
    data.num_point_2 = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_delta_robot/num_point';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f69ff62ad97702372d7a82c71cf0ac27';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 num_point_1
    int64 num_point_2
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new num_point(null);
    if (msg.num_point_1 !== undefined) {
      resolved.num_point_1 = msg.num_point_1;
    }
    else {
      resolved.num_point_1 = 0
    }

    if (msg.num_point_2 !== undefined) {
      resolved.num_point_2 = msg.num_point_2;
    }
    else {
      resolved.num_point_2 = 0
    }

    return resolved;
    }
};

module.exports = num_point;
