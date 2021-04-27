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

class CubeDefString {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.CubeDefString = null;
    }
    else {
      if (initObj.hasOwnProperty('CubeDefString')) {
        this.CubeDefString = initObj.CubeDefString
      }
      else {
        this.CubeDefString = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CubeDefString
    // Serialize message field [CubeDefString]
    bufferOffset = _serializer.string(obj.CubeDefString, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CubeDefString
    let len;
    let data = new CubeDefString(null);
    // Deserialize message field [CubeDefString]
    data.CubeDefString = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.CubeDefString.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'twophase_solver_ros/CubeDefString';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a3ff4760f41bf547735696b3857691d0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string CubeDefString
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CubeDefString(null);
    if (msg.CubeDefString !== undefined) {
      resolved.CubeDefString = msg.CubeDefString;
    }
    else {
      resolved.CubeDefString = ''
    }

    return resolved;
    }
};

module.exports = CubeDefString;
