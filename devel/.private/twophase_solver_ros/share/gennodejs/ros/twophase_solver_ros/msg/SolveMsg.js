// Auto-generated. Do not edit!

// (in-package twophase_solver_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SolveMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.movecount = null;
      this.SolveString = null;
    }
    else {
      if (initObj.hasOwnProperty('movecount')) {
        this.movecount = initObj.movecount
      }
      else {
        this.movecount = 0;
      }
      if (initObj.hasOwnProperty('SolveString')) {
        this.SolveString = initObj.SolveString
      }
      else {
        this.SolveString = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SolveMsg
    // Serialize message field [movecount]
    bufferOffset = _serializer.int64(obj.movecount, buffer, bufferOffset);
    // Serialize message field [SolveString]
    bufferOffset = _serializer.string(obj.SolveString, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SolveMsg
    let len;
    let data = new SolveMsg(null);
    // Deserialize message field [movecount]
    data.movecount = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [SolveString]
    data.SolveString = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.SolveString.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'twophase_solver_ros/SolveMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f8b96427a91926b7a010189fd3281ee2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 movecount
    string SolveString
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SolveMsg(null);
    if (msg.movecount !== undefined) {
      resolved.movecount = msg.movecount;
    }
    else {
      resolved.movecount = 0
    }

    if (msg.SolveString !== undefined) {
      resolved.SolveString = msg.SolveString;
    }
    else {
      resolved.SolveString = ''
    }

    return resolved;
    }
};

module.exports = SolveMsg;
