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

class PositionArm {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.base_brazo1 = null;
      this.base_brazo2 = null;
      this.base_brazo3 = null;
      this.codo1_a = null;
      this.codo1_b = null;
      this.codo2_a = null;
      this.codo2_b = null;
      this.codo3_a = null;
      this.codo3_b = null;
      this.act_x = null;
      this.act_y = null;
      this.act_z = null;
    }
    else {
      if (initObj.hasOwnProperty('base_brazo1')) {
        this.base_brazo1 = initObj.base_brazo1
      }
      else {
        this.base_brazo1 = 0.0;
      }
      if (initObj.hasOwnProperty('base_brazo2')) {
        this.base_brazo2 = initObj.base_brazo2
      }
      else {
        this.base_brazo2 = 0.0;
      }
      if (initObj.hasOwnProperty('base_brazo3')) {
        this.base_brazo3 = initObj.base_brazo3
      }
      else {
        this.base_brazo3 = 0.0;
      }
      if (initObj.hasOwnProperty('codo1_a')) {
        this.codo1_a = initObj.codo1_a
      }
      else {
        this.codo1_a = 0.0;
      }
      if (initObj.hasOwnProperty('codo1_b')) {
        this.codo1_b = initObj.codo1_b
      }
      else {
        this.codo1_b = 0.0;
      }
      if (initObj.hasOwnProperty('codo2_a')) {
        this.codo2_a = initObj.codo2_a
      }
      else {
        this.codo2_a = 0.0;
      }
      if (initObj.hasOwnProperty('codo2_b')) {
        this.codo2_b = initObj.codo2_b
      }
      else {
        this.codo2_b = 0.0;
      }
      if (initObj.hasOwnProperty('codo3_a')) {
        this.codo3_a = initObj.codo3_a
      }
      else {
        this.codo3_a = 0.0;
      }
      if (initObj.hasOwnProperty('codo3_b')) {
        this.codo3_b = initObj.codo3_b
      }
      else {
        this.codo3_b = 0.0;
      }
      if (initObj.hasOwnProperty('act_x')) {
        this.act_x = initObj.act_x
      }
      else {
        this.act_x = 0.0;
      }
      if (initObj.hasOwnProperty('act_y')) {
        this.act_y = initObj.act_y
      }
      else {
        this.act_y = 0.0;
      }
      if (initObj.hasOwnProperty('act_z')) {
        this.act_z = initObj.act_z
      }
      else {
        this.act_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PositionArm
    // Serialize message field [base_brazo1]
    bufferOffset = _serializer.float64(obj.base_brazo1, buffer, bufferOffset);
    // Serialize message field [base_brazo2]
    bufferOffset = _serializer.float64(obj.base_brazo2, buffer, bufferOffset);
    // Serialize message field [base_brazo3]
    bufferOffset = _serializer.float64(obj.base_brazo3, buffer, bufferOffset);
    // Serialize message field [codo1_a]
    bufferOffset = _serializer.float64(obj.codo1_a, buffer, bufferOffset);
    // Serialize message field [codo1_b]
    bufferOffset = _serializer.float64(obj.codo1_b, buffer, bufferOffset);
    // Serialize message field [codo2_a]
    bufferOffset = _serializer.float64(obj.codo2_a, buffer, bufferOffset);
    // Serialize message field [codo2_b]
    bufferOffset = _serializer.float64(obj.codo2_b, buffer, bufferOffset);
    // Serialize message field [codo3_a]
    bufferOffset = _serializer.float64(obj.codo3_a, buffer, bufferOffset);
    // Serialize message field [codo3_b]
    bufferOffset = _serializer.float64(obj.codo3_b, buffer, bufferOffset);
    // Serialize message field [act_x]
    bufferOffset = _serializer.float64(obj.act_x, buffer, bufferOffset);
    // Serialize message field [act_y]
    bufferOffset = _serializer.float64(obj.act_y, buffer, bufferOffset);
    // Serialize message field [act_z]
    bufferOffset = _serializer.float64(obj.act_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PositionArm
    let len;
    let data = new PositionArm(null);
    // Deserialize message field [base_brazo1]
    data.base_brazo1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [base_brazo2]
    data.base_brazo2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [base_brazo3]
    data.base_brazo3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [codo1_a]
    data.codo1_a = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [codo1_b]
    data.codo1_b = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [codo2_a]
    data.codo2_a = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [codo2_b]
    data.codo2_b = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [codo3_a]
    data.codo3_a = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [codo3_b]
    data.codo3_b = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [act_x]
    data.act_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [act_y]
    data.act_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [act_z]
    data.act_z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_delta_robot/PositionArm';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd2558a79329645ec1605ebea813b04a6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 base_brazo1
    float64 base_brazo2
    float64 base_brazo3
    float64 codo1_a
    float64 codo1_b
    float64 codo2_a
    float64 codo2_b
    float64 codo3_a
    float64 codo3_b
    float64 act_x
    float64 act_y
    float64 act_z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PositionArm(null);
    if (msg.base_brazo1 !== undefined) {
      resolved.base_brazo1 = msg.base_brazo1;
    }
    else {
      resolved.base_brazo1 = 0.0
    }

    if (msg.base_brazo2 !== undefined) {
      resolved.base_brazo2 = msg.base_brazo2;
    }
    else {
      resolved.base_brazo2 = 0.0
    }

    if (msg.base_brazo3 !== undefined) {
      resolved.base_brazo3 = msg.base_brazo3;
    }
    else {
      resolved.base_brazo3 = 0.0
    }

    if (msg.codo1_a !== undefined) {
      resolved.codo1_a = msg.codo1_a;
    }
    else {
      resolved.codo1_a = 0.0
    }

    if (msg.codo1_b !== undefined) {
      resolved.codo1_b = msg.codo1_b;
    }
    else {
      resolved.codo1_b = 0.0
    }

    if (msg.codo2_a !== undefined) {
      resolved.codo2_a = msg.codo2_a;
    }
    else {
      resolved.codo2_a = 0.0
    }

    if (msg.codo2_b !== undefined) {
      resolved.codo2_b = msg.codo2_b;
    }
    else {
      resolved.codo2_b = 0.0
    }

    if (msg.codo3_a !== undefined) {
      resolved.codo3_a = msg.codo3_a;
    }
    else {
      resolved.codo3_a = 0.0
    }

    if (msg.codo3_b !== undefined) {
      resolved.codo3_b = msg.codo3_b;
    }
    else {
      resolved.codo3_b = 0.0
    }

    if (msg.act_x !== undefined) {
      resolved.act_x = msg.act_x;
    }
    else {
      resolved.act_x = 0.0
    }

    if (msg.act_y !== undefined) {
      resolved.act_y = msg.act_y;
    }
    else {
      resolved.act_y = 0.0
    }

    if (msg.act_z !== undefined) {
      resolved.act_z = msg.act_z;
    }
    else {
      resolved.act_z = 0.0
    }

    return resolved;
    }
};

module.exports = PositionArm;
