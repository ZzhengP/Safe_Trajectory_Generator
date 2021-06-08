// Auto-generated. Do not edit!

// (in-package panda_mpc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class trajectoryMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.next_point = null;
      this.next_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
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
    // Serializes a message object of type trajectoryMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [next_point]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.next_point, buffer, bufferOffset);
    // Serialize message field [next_vel]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.next_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type trajectoryMsg
    let len;
    let data = new trajectoryMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [next_point]
    data.next_point = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [next_vel]
    data.next_vel = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'panda_mpc/trajectoryMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87e2c8c7845fec4f6ddc003e2af217c5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    geometry_msgs/Vector3 next_point
    geometry_msgs/Twist next_vel
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
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
    const resolved = new trajectoryMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

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

module.exports = trajectoryMsg;
