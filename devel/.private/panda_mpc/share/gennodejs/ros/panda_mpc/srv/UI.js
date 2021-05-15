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

class UIRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.play_traj = null;
      this.jog_robot = null;
      this.publish_traj = null;
      this.build_traj = null;
      this.positioning_ = null;
      this.p_gains_ = null;
      this.d_gains_ = null;
      this.move_signal_ = null;
      this.tune_gain = null;
      this.amplitude = null;
      this.axis = null;
      this.exit_ = null;
      this.distance_to_contact_ = null;
      this.fake_distance_ = null;
    }
    else {
      if (initObj.hasOwnProperty('play_traj')) {
        this.play_traj = initObj.play_traj
      }
      else {
        this.play_traj = false;
      }
      if (initObj.hasOwnProperty('jog_robot')) {
        this.jog_robot = initObj.jog_robot
      }
      else {
        this.jog_robot = false;
      }
      if (initObj.hasOwnProperty('publish_traj')) {
        this.publish_traj = initObj.publish_traj
      }
      else {
        this.publish_traj = false;
      }
      if (initObj.hasOwnProperty('build_traj')) {
        this.build_traj = initObj.build_traj
      }
      else {
        this.build_traj = false;
      }
      if (initObj.hasOwnProperty('positioning_')) {
        this.positioning_ = initObj.positioning_
      }
      else {
        this.positioning_ = false;
      }
      if (initObj.hasOwnProperty('p_gains_')) {
        this.p_gains_ = initObj.p_gains_
      }
      else {
        this.p_gains_ = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('d_gains_')) {
        this.d_gains_ = initObj.d_gains_
      }
      else {
        this.d_gains_ = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('move_signal_')) {
        this.move_signal_ = initObj.move_signal_
      }
      else {
        this.move_signal_ = false;
      }
      if (initObj.hasOwnProperty('tune_gain')) {
        this.tune_gain = initObj.tune_gain
      }
      else {
        this.tune_gain = false;
      }
      if (initObj.hasOwnProperty('amplitude')) {
        this.amplitude = initObj.amplitude
      }
      else {
        this.amplitude = 0.0;
      }
      if (initObj.hasOwnProperty('axis')) {
        this.axis = initObj.axis
      }
      else {
        this.axis = 0;
      }
      if (initObj.hasOwnProperty('exit_')) {
        this.exit_ = initObj.exit_
      }
      else {
        this.exit_ = false;
      }
      if (initObj.hasOwnProperty('distance_to_contact_')) {
        this.distance_to_contact_ = initObj.distance_to_contact_
      }
      else {
        this.distance_to_contact_ = 0.0;
      }
      if (initObj.hasOwnProperty('fake_distance_')) {
        this.fake_distance_ = initObj.fake_distance_
      }
      else {
        this.fake_distance_ = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UIRequest
    // Serialize message field [play_traj]
    bufferOffset = _serializer.bool(obj.play_traj, buffer, bufferOffset);
    // Serialize message field [jog_robot]
    bufferOffset = _serializer.bool(obj.jog_robot, buffer, bufferOffset);
    // Serialize message field [publish_traj]
    bufferOffset = _serializer.bool(obj.publish_traj, buffer, bufferOffset);
    // Serialize message field [build_traj]
    bufferOffset = _serializer.bool(obj.build_traj, buffer, bufferOffset);
    // Serialize message field [positioning_]
    bufferOffset = _serializer.bool(obj.positioning_, buffer, bufferOffset);
    // Serialize message field [p_gains_]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.p_gains_, buffer, bufferOffset);
    // Serialize message field [d_gains_]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.d_gains_, buffer, bufferOffset);
    // Serialize message field [move_signal_]
    bufferOffset = _serializer.bool(obj.move_signal_, buffer, bufferOffset);
    // Serialize message field [tune_gain]
    bufferOffset = _serializer.bool(obj.tune_gain, buffer, bufferOffset);
    // Serialize message field [amplitude]
    bufferOffset = _serializer.float64(obj.amplitude, buffer, bufferOffset);
    // Serialize message field [axis]
    bufferOffset = _serializer.int64(obj.axis, buffer, bufferOffset);
    // Serialize message field [exit_]
    bufferOffset = _serializer.bool(obj.exit_, buffer, bufferOffset);
    // Serialize message field [distance_to_contact_]
    bufferOffset = _serializer.float64(obj.distance_to_contact_, buffer, bufferOffset);
    // Serialize message field [fake_distance_]
    bufferOffset = _serializer.bool(obj.fake_distance_, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UIRequest
    let len;
    let data = new UIRequest(null);
    // Deserialize message field [play_traj]
    data.play_traj = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [jog_robot]
    data.jog_robot = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [publish_traj]
    data.publish_traj = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [build_traj]
    data.build_traj = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [positioning_]
    data.positioning_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [p_gains_]
    data.p_gains_ = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [d_gains_]
    data.d_gains_ = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [move_signal_]
    data.move_signal_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tune_gain]
    data.tune_gain = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [amplitude]
    data.amplitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [axis]
    data.axis = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [exit_]
    data.exit_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [distance_to_contact_]
    data.distance_to_contact_ = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [fake_distance_]
    data.fake_distance_ = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 129;
  }

  static datatype() {
    // Returns string type for a service object
    return 'panda_mpc/UIRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c014034d544e11b0948e3346a66ba7c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool play_traj
    bool jog_robot
    bool publish_traj 
    bool build_traj
    bool positioning_
    geometry_msgs/Twist p_gains_
    geometry_msgs/Twist d_gains_
    bool move_signal_
    bool tune_gain
    float64 amplitude
    int64 axis
    bool exit_
    float64 distance_to_contact_
    bool fake_distance_
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UIRequest(null);
    if (msg.play_traj !== undefined) {
      resolved.play_traj = msg.play_traj;
    }
    else {
      resolved.play_traj = false
    }

    if (msg.jog_robot !== undefined) {
      resolved.jog_robot = msg.jog_robot;
    }
    else {
      resolved.jog_robot = false
    }

    if (msg.publish_traj !== undefined) {
      resolved.publish_traj = msg.publish_traj;
    }
    else {
      resolved.publish_traj = false
    }

    if (msg.build_traj !== undefined) {
      resolved.build_traj = msg.build_traj;
    }
    else {
      resolved.build_traj = false
    }

    if (msg.positioning_ !== undefined) {
      resolved.positioning_ = msg.positioning_;
    }
    else {
      resolved.positioning_ = false
    }

    if (msg.p_gains_ !== undefined) {
      resolved.p_gains_ = geometry_msgs.msg.Twist.Resolve(msg.p_gains_)
    }
    else {
      resolved.p_gains_ = new geometry_msgs.msg.Twist()
    }

    if (msg.d_gains_ !== undefined) {
      resolved.d_gains_ = geometry_msgs.msg.Twist.Resolve(msg.d_gains_)
    }
    else {
      resolved.d_gains_ = new geometry_msgs.msg.Twist()
    }

    if (msg.move_signal_ !== undefined) {
      resolved.move_signal_ = msg.move_signal_;
    }
    else {
      resolved.move_signal_ = false
    }

    if (msg.tune_gain !== undefined) {
      resolved.tune_gain = msg.tune_gain;
    }
    else {
      resolved.tune_gain = false
    }

    if (msg.amplitude !== undefined) {
      resolved.amplitude = msg.amplitude;
    }
    else {
      resolved.amplitude = 0.0
    }

    if (msg.axis !== undefined) {
      resolved.axis = msg.axis;
    }
    else {
      resolved.axis = 0
    }

    if (msg.exit_ !== undefined) {
      resolved.exit_ = msg.exit_;
    }
    else {
      resolved.exit_ = false
    }

    if (msg.distance_to_contact_ !== undefined) {
      resolved.distance_to_contact_ = msg.distance_to_contact_;
    }
    else {
      resolved.distance_to_contact_ = 0.0
    }

    if (msg.fake_distance_ !== undefined) {
      resolved.fake_distance_ = msg.fake_distance_;
    }
    else {
      resolved.fake_distance_ = false
    }

    return resolved;
    }
};

class UIResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UIResponse
    // Serialize message field [result]
    bufferOffset = _serializer.bool(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UIResponse
    let len;
    let data = new UIResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'panda_mpc/UIResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eb13ac1f1354ccecb7941ee8fa2192e8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UIResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = false
    }

    return resolved;
    }
};

module.exports = {
  Request: UIRequest,
  Response: UIResponse,
  md5sum() { return '2e1bdac3ef57deb56fae70ed4385f616'; },
  datatype() { return 'panda_mpc/UI'; }
};
