// Auto-generated. Do not edit!

// (in-package panda_traj.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nav_msgs = _finder('nav_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class PublishTraj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose_array_ = null;
      this.path_ros_ = null;
    }
    else {
      if (initObj.hasOwnProperty('pose_array_')) {
        this.pose_array_ = initObj.pose_array_
      }
      else {
        this.pose_array_ = new geometry_msgs.msg.PoseArray();
      }
      if (initObj.hasOwnProperty('path_ros_')) {
        this.path_ros_ = initObj.path_ros_
      }
      else {
        this.path_ros_ = new nav_msgs.msg.Path();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PublishTraj
    // Serialize message field [pose_array_]
    bufferOffset = geometry_msgs.msg.PoseArray.serialize(obj.pose_array_, buffer, bufferOffset);
    // Serialize message field [path_ros_]
    bufferOffset = nav_msgs.msg.Path.serialize(obj.path_ros_, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PublishTraj
    let len;
    let data = new PublishTraj(null);
    // Deserialize message field [pose_array_]
    data.pose_array_ = geometry_msgs.msg.PoseArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [path_ros_]
    data.path_ros_ = nav_msgs.msg.Path.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseArray.getMessageSize(object.pose_array_);
    length += nav_msgs.msg.Path.getMessageSize(object.path_ros_);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'panda_traj/PublishTraj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '23616706435ed56017355b3bc7eafc99';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/PoseArray pose_array_
    nav_msgs/Path path_ros_
    ================================================================================
    MSG: geometry_msgs/PoseArray
    # An array of poses with a header for global reference.
    
    Header header
    
    Pose[] poses
    
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
    
    ================================================================================
    MSG: nav_msgs/Path
    #An array of poses that represents a Path for a robot to follow
    Header header
    geometry_msgs/PoseStamped[] poses
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PublishTraj(null);
    if (msg.pose_array_ !== undefined) {
      resolved.pose_array_ = geometry_msgs.msg.PoseArray.Resolve(msg.pose_array_)
    }
    else {
      resolved.pose_array_ = new geometry_msgs.msg.PoseArray()
    }

    if (msg.path_ros_ !== undefined) {
      resolved.path_ros_ = nav_msgs.msg.Path.Resolve(msg.path_ros_)
    }
    else {
      resolved.path_ros_ = new nav_msgs.msg.Path()
    }

    return resolved;
    }
};

module.exports = PublishTraj;
