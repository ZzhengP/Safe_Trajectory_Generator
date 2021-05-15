// Auto-generated. Do not edit!

// (in-package panda_traj.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class TrajProperties {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.play_traj_ = null;
      this.jogging_ = null;
      this.gain_tunning_ = null;
      this.move_ = null;
      this.index_ = null;
      this.amplitude = null;
      this.X_curr_ = null;
      this.X_des_jog_ = null;
    }
    else {
      if (initObj.hasOwnProperty('play_traj_')) {
        this.play_traj_ = initObj.play_traj_
      }
      else {
        this.play_traj_ = false;
      }
      if (initObj.hasOwnProperty('jogging_')) {
        this.jogging_ = initObj.jogging_
      }
      else {
        this.jogging_ = false;
      }
      if (initObj.hasOwnProperty('gain_tunning_')) {
        this.gain_tunning_ = initObj.gain_tunning_
      }
      else {
        this.gain_tunning_ = false;
      }
      if (initObj.hasOwnProperty('move_')) {
        this.move_ = initObj.move_
      }
      else {
        this.move_ = false;
      }
      if (initObj.hasOwnProperty('index_')) {
        this.index_ = initObj.index_
      }
      else {
        this.index_ = 0;
      }
      if (initObj.hasOwnProperty('amplitude')) {
        this.amplitude = initObj.amplitude
      }
      else {
        this.amplitude = 0.0;
      }
      if (initObj.hasOwnProperty('X_curr_')) {
        this.X_curr_ = initObj.X_curr_
      }
      else {
        this.X_curr_ = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('X_des_jog_')) {
        this.X_des_jog_ = initObj.X_des_jog_
      }
      else {
        this.X_des_jog_ = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajProperties
    // Serialize message field [play_traj_]
    bufferOffset = _serializer.bool(obj.play_traj_, buffer, bufferOffset);
    // Serialize message field [jogging_]
    bufferOffset = _serializer.bool(obj.jogging_, buffer, bufferOffset);
    // Serialize message field [gain_tunning_]
    bufferOffset = _serializer.bool(obj.gain_tunning_, buffer, bufferOffset);
    // Serialize message field [move_]
    bufferOffset = _serializer.bool(obj.move_, buffer, bufferOffset);
    // Serialize message field [index_]
    bufferOffset = _serializer.int64(obj.index_, buffer, bufferOffset);
    // Serialize message field [amplitude]
    bufferOffset = _serializer.float64(obj.amplitude, buffer, bufferOffset);
    // Serialize message field [X_curr_]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.X_curr_, buffer, bufferOffset);
    // Serialize message field [X_des_jog_]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.X_des_jog_, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajProperties
    let len;
    let data = new TrajProperties(null);
    // Deserialize message field [play_traj_]
    data.play_traj_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [jogging_]
    data.jogging_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gain_tunning_]
    data.gain_tunning_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [move_]
    data.move_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [index_]
    data.index_ = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [amplitude]
    data.amplitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [X_curr_]
    data.X_curr_ = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [X_des_jog_]
    data.X_des_jog_ = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 132;
  }

  static datatype() {
    // Returns string type for a message object
    return 'panda_traj/TrajProperties';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8fb34236d88ea1e31629703f4e635b92';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool play_traj_
    bool jogging_
    bool gain_tunning_
    bool move_
    int64 index_
    float64 amplitude
    geometry_msgs/Pose X_curr_
    geometry_msgs/Pose X_des_jog_
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajProperties(null);
    if (msg.play_traj_ !== undefined) {
      resolved.play_traj_ = msg.play_traj_;
    }
    else {
      resolved.play_traj_ = false
    }

    if (msg.jogging_ !== undefined) {
      resolved.jogging_ = msg.jogging_;
    }
    else {
      resolved.jogging_ = false
    }

    if (msg.gain_tunning_ !== undefined) {
      resolved.gain_tunning_ = msg.gain_tunning_;
    }
    else {
      resolved.gain_tunning_ = false
    }

    if (msg.move_ !== undefined) {
      resolved.move_ = msg.move_;
    }
    else {
      resolved.move_ = false
    }

    if (msg.index_ !== undefined) {
      resolved.index_ = msg.index_;
    }
    else {
      resolved.index_ = 0
    }

    if (msg.amplitude !== undefined) {
      resolved.amplitude = msg.amplitude;
    }
    else {
      resolved.amplitude = 0.0
    }

    if (msg.X_curr_ !== undefined) {
      resolved.X_curr_ = geometry_msgs.msg.Pose.Resolve(msg.X_curr_)
    }
    else {
      resolved.X_curr_ = new geometry_msgs.msg.Pose()
    }

    if (msg.X_des_jog_ !== undefined) {
      resolved.X_des_jog_ = geometry_msgs.msg.Pose.Resolve(msg.X_des_jog_)
    }
    else {
      resolved.X_des_jog_ = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = TrajProperties;
