// Auto-generated. Do not edit!

// (in-package panda_mpc.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class UpdateTrajectoryNextPointRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.next_point = null;
      this.next_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('next_point')) {
        this.next_point = initObj.next_point
      }
      else {
        this.next_point = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('next_vel')) {
        this.next_vel = initObj.next_vel
      }
      else {
        this.next_vel = new geometry_msgs.msg.Twist();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UpdateTrajectoryNextPointRequest
    // Serialize message field [next_point]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.next_point, buffer, bufferOffset);
    // Serialize message field [next_vel]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.next_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UpdateTrajectoryNextPointRequest
    let len;
    let data = new UpdateTrajectoryNextPointRequest(null);
    // Deserialize message field [next_point]
    data.next_point = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [next_vel]
    data.next_vel = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 72;
  }

  static datatype() {
    // Returns string type for a service object
    return 'panda_mpc/UpdateTrajectoryNextPointRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2685428bccce90adb95a24e6e6228b8f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Vector3 next_point
    geometry_msgs/Twist next_vel
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UpdateTrajectoryNextPointRequest(null);
    if (msg.next_point !== undefined) {
      resolved.next_point = geometry_msgs.msg.Vector3.Resolve(msg.next_point)
    }
    else {
      resolved.next_point = new geometry_msgs.msg.Vector3()
    }

    if (msg.next_vel !== undefined) {
      resolved.next_vel = geometry_msgs.msg.Twist.Resolve(msg.next_vel)
    }
    else {
      resolved.next_vel = new geometry_msgs.msg.Twist()
    }

    return resolved;
    }
};

class UpdateTrajectoryNextPointResponse {
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
    // Serializes a message object of type UpdateTrajectoryNextPointResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UpdateTrajectoryNextPointResponse
    let len;
    let data = new UpdateTrajectoryNextPointResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'panda_mpc/UpdateTrajectoryNextPointResponse';
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
    const resolved = new UpdateTrajectoryNextPointResponse(null);
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
  Request: UpdateTrajectoryNextPointRequest,
  Response: UpdateTrajectoryNextPointResponse,
  md5sum() { return '07d2a1fe5433c3b25d4df2bdf8fc4f1d'; },
  datatype() { return 'panda_mpc/UpdateTrajectoryNextPoint'; }
};
