// Auto-generated. Do not edit!

// (in-package twophase_solver_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SolverRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.defstr = null;
    }
    else {
      if (initObj.hasOwnProperty('defstr')) {
        this.defstr = initObj.defstr
      }
      else {
        this.defstr = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SolverRequest
    // Serialize message field [defstr]
    bufferOffset = _serializer.string(obj.defstr, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SolverRequest
    let len;
    let data = new SolverRequest(null);
    // Deserialize message field [defstr]
    data.defstr = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.defstr.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'twophase_solver_ros/SolverRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'def6775a2c5fbf7096579b2d03edfae5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string defstr
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SolverRequest(null);
    if (msg.defstr !== undefined) {
      resolved.defstr = msg.defstr;
    }
    else {
      resolved.defstr = ''
    }

    return resolved;
    }
};

class SolverResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.movecount = null;
      this.solution = null;
    }
    else {
      if (initObj.hasOwnProperty('movecount')) {
        this.movecount = initObj.movecount
      }
      else {
        this.movecount = 0;
      }
      if (initObj.hasOwnProperty('solution')) {
        this.solution = initObj.solution
      }
      else {
        this.solution = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SolverResponse
    // Serialize message field [movecount]
    bufferOffset = _serializer.int64(obj.movecount, buffer, bufferOffset);
    // Serialize message field [solution]
    bufferOffset = _serializer.string(obj.solution, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SolverResponse
    let len;
    let data = new SolverResponse(null);
    // Deserialize message field [movecount]
    data.movecount = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [solution]
    data.solution = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.solution.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'twophase_solver_ros/SolverResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '241bf767e2780f284a6ad3e62879583f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 movecount
    string solution
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SolverResponse(null);
    if (msg.movecount !== undefined) {
      resolved.movecount = msg.movecount;
    }
    else {
      resolved.movecount = 0
    }

    if (msg.solution !== undefined) {
      resolved.solution = msg.solution;
    }
    else {
      resolved.solution = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SolverRequest,
  Response: SolverResponse,
  md5sum() { return '3dd62463a4927de8181123eea48c8934'; },
  datatype() { return 'twophase_solver_ros/Solver'; }
};
