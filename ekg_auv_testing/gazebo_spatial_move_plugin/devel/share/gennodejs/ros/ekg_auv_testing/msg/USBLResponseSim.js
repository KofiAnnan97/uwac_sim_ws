// Auto-generated. Do not edit!

// (in-package ekg_auv_testing.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class USBLResponseSim {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.transceiverID = null;
      this.responseID = null;
      this.transceiverModelName = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('transceiverID')) {
        this.transceiverID = initObj.transceiverID
      }
      else {
        this.transceiverID = 0;
      }
      if (initObj.hasOwnProperty('responseID')) {
        this.responseID = initObj.responseID
      }
      else {
        this.responseID = 0;
      }
      if (initObj.hasOwnProperty('transceiverModelName')) {
        this.transceiverModelName = initObj.transceiverModelName
      }
      else {
        this.transceiverModelName = '';
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type USBLResponseSim
    // Serialize message field [transceiverID]
    bufferOffset = _serializer.int32(obj.transceiverID, buffer, bufferOffset);
    // Serialize message field [responseID]
    bufferOffset = _serializer.int32(obj.responseID, buffer, bufferOffset);
    // Serialize message field [transceiverModelName]
    bufferOffset = _serializer.string(obj.transceiverModelName, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _serializer.string(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type USBLResponseSim
    let len;
    let data = new USBLResponseSim(null);
    // Deserialize message field [transceiverID]
    data.transceiverID = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [responseID]
    data.responseID = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [transceiverModelName]
    data.transceiverModelName = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.transceiverModelName);
    length += _getByteLength(object.data);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ekg_auv_testing/USBLResponseSim';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '23b647a5a32e3bbb7ba751a1824c350f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 transceiverID
    int32 responseID
    string transceiverModelName
    string data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new USBLResponseSim(null);
    if (msg.transceiverID !== undefined) {
      resolved.transceiverID = msg.transceiverID;
    }
    else {
      resolved.transceiverID = 0
    }

    if (msg.responseID !== undefined) {
      resolved.responseID = msg.responseID;
    }
    else {
      resolved.responseID = 0
    }

    if (msg.transceiverModelName !== undefined) {
      resolved.transceiverModelName = msg.transceiverModelName;
    }
    else {
      resolved.transceiverModelName = ''
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = ''
    }

    return resolved;
    }
};

module.exports = USBLResponseSim;
