// Auto-generated. Do not edit!

// (in-package panda_traj.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class UpdateTrajectoryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.csv_traj_path = null;
      this.verbose = null;
    }
    else {
      if (initObj.hasOwnProperty('csv_traj_path')) {
        this.csv_traj_path = initObj.csv_traj_path
      }
      else {
        this.csv_traj_path = '';
      }
      if (initObj.hasOwnProperty('verbose')) {
        this.verbose = initObj.verbose
      }
      else {
        this.verbose = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UpdateTrajectoryRequest
    // Serialize message field [csv_traj_path]
    bufferOffset = _serializer.string(obj.csv_traj_path, buffer, bufferOffset);
    // Serialize message field [verbose]
    bufferOffset = _serializer.bool(obj.verbose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UpdateTrajectoryRequest
    let len;
    let data = new UpdateTrajectoryRequest(null);
    // Deserialize message field [csv_traj_path]
    data.csv_traj_path = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [verbose]
    data.verbose = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.csv_traj_path.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'panda_traj/UpdateTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6853df9f42572d60890fd8fa81729a5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string csv_traj_path
    bool verbose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UpdateTrajectoryRequest(null);
    if (msg.csv_traj_path !== undefined) {
      resolved.csv_traj_path = msg.csv_traj_path;
    }
    else {
      resolved.csv_traj_path = ''
    }

    if (msg.verbose !== undefined) {
      resolved.verbose = msg.verbose;
    }
    else {
      resolved.verbose = false
    }

    return resolved;
    }
};

class UpdateTrajectoryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UpdateTrajectoryResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UpdateTrajectoryResponse
    let len;
    let data = new UpdateTrajectoryResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'panda_traj/UpdateTrajectoryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UpdateTrajectoryResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: UpdateTrajectoryRequest,
  Response: UpdateTrajectoryResponse,
  md5sum() { return 'df273a5e598ca3c1de49e8c78524e55a'; },
  datatype() { return 'panda_traj/UpdateTrajectory'; }
};
