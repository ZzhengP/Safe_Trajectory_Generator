// Auto-generated. Do not edit!

// (in-package panda_mpc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PandaRunMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.vmax_ec = null;
      this.t_traj_curr = null;
      this.Xd_traj = null;
      this.Xdd_traj = null;
      this.X_err = null;
      this.Xd_control = null;
      this.qd_des = null;
      this.play_traj_ = null;
      this.positioning_ = null;
      this.tune_gains_ = null;
      this.distance_to_contact_ = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('vmax_ec')) {
        this.vmax_ec = initObj.vmax_ec
      }
      else {
        this.vmax_ec = 0.0;
      }
      if (initObj.hasOwnProperty('t_traj_curr')) {
        this.t_traj_curr = initObj.t_traj_curr
      }
      else {
        this.t_traj_curr = 0.0;
      }
      if (initObj.hasOwnProperty('Xd_traj')) {
        this.Xd_traj = initObj.Xd_traj
      }
      else {
        this.Xd_traj = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('Xdd_traj')) {
        this.Xdd_traj = initObj.Xdd_traj
      }
      else {
        this.Xdd_traj = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('X_err')) {
        this.X_err = initObj.X_err
      }
      else {
        this.X_err = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('Xd_control')) {
        this.Xd_control = initObj.Xd_control
      }
      else {
        this.Xd_control = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('qd_des')) {
        this.qd_des = initObj.qd_des
      }
      else {
        this.qd_des = new sensor_msgs.msg.JointState();
      }
      if (initObj.hasOwnProperty('play_traj_')) {
        this.play_traj_ = initObj.play_traj_
      }
      else {
        this.play_traj_ = false;
      }
      if (initObj.hasOwnProperty('positioning_')) {
        this.positioning_ = initObj.positioning_
      }
      else {
        this.positioning_ = false;
      }
      if (initObj.hasOwnProperty('tune_gains_')) {
        this.tune_gains_ = initObj.tune_gains_
      }
      else {
        this.tune_gains_ = false;
      }
      if (initObj.hasOwnProperty('distance_to_contact_')) {
        this.distance_to_contact_ = initObj.distance_to_contact_
      }
      else {
        this.distance_to_contact_ = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PandaRunMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [vmax_ec]
    bufferOffset = _serializer.float64(obj.vmax_ec, buffer, bufferOffset);
    // Serialize message field [t_traj_curr]
    bufferOffset = _serializer.float64(obj.t_traj_curr, buffer, bufferOffset);
    // Serialize message field [Xd_traj]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.Xd_traj, buffer, bufferOffset);
    // Serialize message field [Xdd_traj]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.Xdd_traj, buffer, bufferOffset);
    // Serialize message field [X_err]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.X_err, buffer, bufferOffset);
    // Serialize message field [Xd_control]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.Xd_control, buffer, bufferOffset);
    // Serialize message field [qd_des]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.qd_des, buffer, bufferOffset);
    // Serialize message field [play_traj_]
    bufferOffset = _serializer.bool(obj.play_traj_, buffer, bufferOffset);
    // Serialize message field [positioning_]
    bufferOffset = _serializer.bool(obj.positioning_, buffer, bufferOffset);
    // Serialize message field [tune_gains_]
    bufferOffset = _serializer.bool(obj.tune_gains_, buffer, bufferOffset);
    // Serialize message field [distance_to_contact_]
    bufferOffset = _serializer.float64(obj.distance_to_contact_, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PandaRunMsg
    let len;
    let data = new PandaRunMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [vmax_ec]
    data.vmax_ec = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t_traj_curr]
    data.t_traj_curr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Xd_traj]
    data.Xd_traj = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [Xdd_traj]
    data.Xdd_traj = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [X_err]
    data.X_err = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [Xd_control]
    data.Xd_control = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [qd_des]
    data.qd_des = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    // Deserialize message field [play_traj_]
    data.play_traj_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [positioning_]
    data.positioning_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tune_gains_]
    data.tune_gains_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [distance_to_contact_]
    data.distance_to_contact_ = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += sensor_msgs.msg.JointState.getMessageSize(object.qd_des);
    return length + 219;
  }

  static datatype() {
    // Returns string type for a message object
    return 'panda_mpc/PandaRunMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '978d838c2f0b7635c26f79d5649e9439';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 vmax_ec # [m/s]
    float64 t_traj_curr
    geometry_msgs/Twist Xd_traj
    geometry_msgs/Twist Xdd_traj
    geometry_msgs/Twist X_err
    geometry_msgs/Twist Xd_control
    sensor_msgs/JointState qd_des
    bool play_traj_
    bool positioning_
    bool tune_gains_
    float64 distance_to_contact_
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
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PandaRunMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.vmax_ec !== undefined) {
      resolved.vmax_ec = msg.vmax_ec;
    }
    else {
      resolved.vmax_ec = 0.0
    }

    if (msg.t_traj_curr !== undefined) {
      resolved.t_traj_curr = msg.t_traj_curr;
    }
    else {
      resolved.t_traj_curr = 0.0
    }

    if (msg.Xd_traj !== undefined) {
      resolved.Xd_traj = geometry_msgs.msg.Twist.Resolve(msg.Xd_traj)
    }
    else {
      resolved.Xd_traj = new geometry_msgs.msg.Twist()
    }

    if (msg.Xdd_traj !== undefined) {
      resolved.Xdd_traj = geometry_msgs.msg.Twist.Resolve(msg.Xdd_traj)
    }
    else {
      resolved.Xdd_traj = new geometry_msgs.msg.Twist()
    }

    if (msg.X_err !== undefined) {
      resolved.X_err = geometry_msgs.msg.Twist.Resolve(msg.X_err)
    }
    else {
      resolved.X_err = new geometry_msgs.msg.Twist()
    }

    if (msg.Xd_control !== undefined) {
      resolved.Xd_control = geometry_msgs.msg.Twist.Resolve(msg.Xd_control)
    }
    else {
      resolved.Xd_control = new geometry_msgs.msg.Twist()
    }

    if (msg.qd_des !== undefined) {
      resolved.qd_des = sensor_msgs.msg.JointState.Resolve(msg.qd_des)
    }
    else {
      resolved.qd_des = new sensor_msgs.msg.JointState()
    }

    if (msg.play_traj_ !== undefined) {
      resolved.play_traj_ = msg.play_traj_;
    }
    else {
      resolved.play_traj_ = false
    }

    if (msg.positioning_ !== undefined) {
      resolved.positioning_ = msg.positioning_;
    }
    else {
      resolved.positioning_ = false
    }

    if (msg.tune_gains_ !== undefined) {
      resolved.tune_gains_ = msg.tune_gains_;
    }
    else {
      resolved.tune_gains_ = false
    }

    if (msg.distance_to_contact_ !== undefined) {
      resolved.distance_to_contact_ = msg.distance_to_contact_;
    }
    else {
      resolved.distance_to_contact_ = 0.0
    }

    return resolved;
    }
};

module.exports = PandaRunMsg;
