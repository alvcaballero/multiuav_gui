var __create = Object.create;
var __defProp = Object.defineProperty;
var __getOwnPropDesc = Object.getOwnPropertyDescriptor;
var __getOwnPropNames = Object.getOwnPropertyNames;
var __getProtoOf = Object.getPrototypeOf;
var __hasOwnProp = Object.prototype.hasOwnProperty;
var __export = (target, all) => {
  for (var name in all)
    __defProp(target, name, { get: all[name], enumerable: true });
};
var __copyProps = (to, from, except, desc) => {
  if (from && typeof from === "object" || typeof from === "function") {
    for (let key of __getOwnPropNames(from))
      if (!__hasOwnProp.call(to, key) && key !== except)
        __defProp(to, key, { get: () => from[key], enumerable: !(desc = __getOwnPropDesc(from, key)) || desc.enumerable });
  }
  return to;
};
var __toESM = (mod, isNodeMode, target) => (target = mod != null ? __create(__getProtoOf(mod)) : {}, __copyProps(
  // If the importer is in node compatibility mode or this is not an ESM
  // file that has been converted to a CommonJS file using a Babel-
  // compatible transform (i.e. "__esModule" has not been set), then set
  // "default" to the CommonJS "module.exports" for node compatibility.
  isNodeMode || !mod || !mod.__esModule ? __defProp(target, "default", { value: mod, enumerable: true }) : target,
  mod
));
var __toCommonJS = (mod) => __copyProps(__defProp({}, "__esModule", { value: true }), mod);

// fbmsglib/src/schema_main.ts
var schema_main_exports = {};
__export(schema_main_exports, {
  fb: () => fb_exports
});
module.exports = __toCommonJS(schema_main_exports);

// fbmsglib/src/fb.ts
var fb_exports = {};
__export(fb_exports, {
  MsgMetadata: () => MsgMetadata,
  MsgWithMetadata: () => MsgWithMetadata,
  RosDuration: () => RosDuration,
  RosTime: () => RosTime,
  aerialcore_common: () => aerialcore_common_exports,
  amrl_msgs: () => amrl_msgs_exports,
  dji_osdk_ros: () => dji_osdk_ros_exports,
  geometry_msgs: () => geometry_msgs_exports,
  mavros_msgs: () => mavros_msgs_exports,
  nav_msgs: () => nav_msgs_exports,
  sensor_msgs: () => sensor_msgs_exports,
  std_msgs: () => std_msgs_exports,
  std_srvs: () => std_srvs_exports
});

// fbmsglib/src/fb/msg-metadata.ts
var flatbuffers = __toESM(require("flatbuffers"));
var MsgMetadata = class _MsgMetadata {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsMsgMetadata(bb, obj) {
    return (obj || new _MsgMetadata()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsMsgMetadata(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new _MsgMetadata()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  type(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  topic(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  static startMsgMetadata(builder) {
    builder.startObject(2);
  }
  static addType(builder, typeOffset) {
    builder.addFieldOffset(0, typeOffset, 0);
  }
  static addTopic(builder, topicOffset) {
    builder.addFieldOffset(1, topicOffset, 0);
  }
  static endMsgMetadata(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createMsgMetadata(builder, typeOffset, topicOffset) {
    _MsgMetadata.startMsgMetadata(builder);
    _MsgMetadata.addType(builder, typeOffset);
    _MsgMetadata.addTopic(builder, topicOffset);
    return _MsgMetadata.endMsgMetadata(builder);
  }
};

// fbmsglib/src/fb/msg-with-metadata.ts
var flatbuffers2 = __toESM(require("flatbuffers"));
var MsgWithMetadata = class _MsgWithMetadata {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsMsgWithMetadata(bb, obj) {
    return (obj || new _MsgWithMetadata()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsMsgWithMetadata(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers2.SIZE_PREFIX_LENGTH);
    return (obj || new _MsgWithMetadata()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startMsgWithMetadata(builder) {
    builder.startObject(1);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static endMsgWithMetadata(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createMsgWithMetadata(builder, _MetadataOffset) {
    _MsgWithMetadata.startMsgWithMetadata(builder);
    _MsgWithMetadata.add_Metadata(builder, _MetadataOffset);
    return _MsgWithMetadata.endMsgWithMetadata(builder);
  }
};

// fbmsglib/src/fb/ros-duration.ts
var RosDuration = class {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  secs() {
    return this.bb.readInt32(this.bb_pos);
  }
  nsecs() {
    return this.bb.readInt32(this.bb_pos + 4);
  }
  static sizeOf() {
    return 8;
  }
  static createRosDuration(builder, secs, nsecs) {
    builder.prep(4, 8);
    builder.writeInt32(nsecs);
    builder.writeInt32(secs);
    return builder.offset();
  }
};

// fbmsglib/src/fb/ros-time.ts
var RosTime = class {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  secs() {
    return this.bb.readUint32(this.bb_pos);
  }
  nsecs() {
    return this.bb.readUint32(this.bb_pos + 4);
  }
  static sizeOf() {
    return 8;
  }
  static createRosTime(builder, secs, nsecs) {
    builder.prep(4, 8);
    builder.writeInt32(nsecs);
    builder.writeInt32(secs);
    return builder.offset();
  }
};

// fbmsglib/src/fb/aerialcore-common.ts
var aerialcore_common_exports = {};
__export(aerialcore_common_exports, {
  ConfigMission: () => ConfigMission,
  ConfigMissionRequest: () => ConfigMissionRequest,
  ConfigMissionResponse: () => ConfigMissionResponse,
  eventRequest: () => eventRequest,
  eventResponse: () => eventResponse,
  eventService: () => eventService
});

// fbmsglib/src/fb/aerialcore-common/config-mission.ts
var flatbuffers8 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/aerialcore-common/config-mission-request.ts
var flatbuffers6 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/sensor-msgs/nav-sat-fix.ts
var flatbuffers5 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/sensor-msgs/nav-sat-status.ts
var flatbuffers3 = __toESM(require("flatbuffers"));
var NavSatStatus = class _NavSatStatus {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsNavSatStatus(bb, obj) {
    return (obj || new _NavSatStatus()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsNavSatStatus(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers3.SIZE_PREFIX_LENGTH);
    return (obj || new _NavSatStatus()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  status() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readInt8(this.bb_pos + offset) : 0;
  }
  service() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint16(this.bb_pos + offset) : 0;
  }
  static startNavSatStatus(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addStatus(builder, status) {
    builder.addFieldInt8(1, status, 0);
  }
  static addService(builder, service) {
    builder.addFieldInt16(2, service, 0);
  }
  static endNavSatStatus(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createNavSatStatus(builder, _MetadataOffset, status, service) {
    _NavSatStatus.startNavSatStatus(builder);
    _NavSatStatus.add_Metadata(builder, _MetadataOffset);
    _NavSatStatus.addStatus(builder, status);
    _NavSatStatus.addService(builder, service);
    return _NavSatStatus.endNavSatStatus(builder);
  }
};

// fbmsglib/src/fb/std-msgs/header.ts
var flatbuffers4 = __toESM(require("flatbuffers"));
var Header = class _Header {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsHeader(bb, obj) {
    return (obj || new _Header()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsHeader(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers4.SIZE_PREFIX_LENGTH);
    return (obj || new _Header()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  seq() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  stamp(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new RosTime()).__init(this.bb_pos + offset, this.bb) : null;
  }
  frameId(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  static startHeader(builder) {
    builder.startObject(4);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addSeq(builder, seq) {
    builder.addFieldInt32(1, seq, 0);
  }
  static addStamp(builder, stampOffset) {
    builder.addFieldStruct(2, stampOffset, 0);
  }
  static addFrameId(builder, frameIdOffset) {
    builder.addFieldOffset(3, frameIdOffset, 0);
  }
  static endHeader(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 8);
    builder.requiredField(offset, 10);
    return offset;
  }
};

// fbmsglib/src/fb/sensor-msgs/nav-sat-fix.ts
var NavSatFix = class _NavSatFix {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsNavSatFix(bb, obj) {
    return (obj || new _NavSatFix()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsNavSatFix(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers5.SIZE_PREFIX_LENGTH);
    return (obj || new _NavSatFix()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  status(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new NavSatStatus()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  latitude() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  longitude() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  altitude() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  positionCovariance(index) {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readFloat64(this.bb.__vector(this.bb_pos + offset) + index * 8) : 0;
  }
  positionCovarianceLength() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  positionCovarianceArray() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? new Float64Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  positionCovarianceType() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  static startNavSatFix(builder) {
    builder.startObject(8);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addStatus(builder, statusOffset) {
    builder.addFieldOffset(2, statusOffset, 0);
  }
  static addLatitude(builder, latitude) {
    builder.addFieldFloat64(3, latitude, 0);
  }
  static addLongitude(builder, longitude) {
    builder.addFieldFloat64(4, longitude, 0);
  }
  static addAltitude(builder, altitude) {
    builder.addFieldFloat64(5, altitude, 0);
  }
  static addPositionCovariance(builder, positionCovarianceOffset) {
    builder.addFieldOffset(6, positionCovarianceOffset, 0);
  }
  static createPositionCovarianceVector(builder, data) {
    builder.startVector(8, data.length, 8);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat64(data[i]);
    }
    return builder.endVector();
  }
  static startPositionCovarianceVector(builder, numElems) {
    builder.startVector(8, numElems, 8);
  }
  static addPositionCovarianceType(builder, positionCovarianceType) {
    builder.addFieldInt8(7, positionCovarianceType, 0);
  }
  static endNavSatFix(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    builder.requiredField(offset, 16);
    return offset;
  }
};

// fbmsglib/src/fb/aerialcore-common/config-mission-request.ts
var ConfigMissionRequest = class _ConfigMissionRequest {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsConfigMissionRequest(bb, obj) {
    return (obj || new _ConfigMissionRequest()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsConfigMissionRequest(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers6.SIZE_PREFIX_LENGTH);
    return (obj || new _ConfigMissionRequest()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  uavId(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  missionId(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  missionType() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  waypoint(index, obj) {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? (obj || new NavSatFix()).__init(this.bb.__indirect(this.bb.__vector(this.bb_pos + offset) + index * 4), this.bb) : null;
  }
  waypointLength() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  radius() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  maxVel() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  idleVel() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  yaw(index) {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  yawLength() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  yawArray() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? new Float32Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  gimbalPitch(index) {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  gimbalPitchLength() {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  gimbalPitchArray() {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? new Float32Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  speed(index) {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  speedLength() {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  speedArray() {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? new Float32Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  yawMode() {
    const offset = this.bb.__offset(this.bb_pos, 24);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  traceMode() {
    const offset = this.bb.__offset(this.bb_pos, 26);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  gimbalPitchMode() {
    const offset = this.bb.__offset(this.bb_pos, 28);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  finishAction() {
    const offset = this.bb.__offset(this.bb_pos, 30);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  commandList(index) {
    const offset = this.bb.__offset(this.bb_pos, 32);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  commandListLength() {
    const offset = this.bb.__offset(this.bb_pos, 32);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  commandListArray() {
    const offset = this.bb.__offset(this.bb_pos, 32);
    return offset ? new Float32Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  commandParameter(index) {
    const offset = this.bb.__offset(this.bb_pos, 34);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  commandParameterLength() {
    const offset = this.bb.__offset(this.bb_pos, 34);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  commandParameterArray() {
    const offset = this.bb.__offset(this.bb_pos, 34);
    return offset ? new Float32Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  static startConfigMissionRequest(builder) {
    builder.startObject(16);
  }
  static addUavId(builder, uavIdOffset) {
    builder.addFieldOffset(0, uavIdOffset, 0);
  }
  static addMissionId(builder, missionIdOffset) {
    builder.addFieldOffset(1, missionIdOffset, 0);
  }
  static addMissionType(builder, missionType) {
    builder.addFieldInt8(2, missionType, 0);
  }
  static addWaypoint(builder, waypointOffset) {
    builder.addFieldOffset(3, waypointOffset, 0);
  }
  static createWaypointVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addOffset(data[i]);
    }
    return builder.endVector();
  }
  static startWaypointVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addRadius(builder, radius) {
    builder.addFieldFloat64(4, radius, 0);
  }
  static addMaxVel(builder, maxVel) {
    builder.addFieldFloat64(5, maxVel, 0);
  }
  static addIdleVel(builder, idleVel) {
    builder.addFieldFloat64(6, idleVel, 0);
  }
  static addYaw(builder, yawOffset) {
    builder.addFieldOffset(7, yawOffset, 0);
  }
  static createYawVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startYawVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addGimbalPitch(builder, gimbalPitchOffset) {
    builder.addFieldOffset(8, gimbalPitchOffset, 0);
  }
  static createGimbalPitchVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startGimbalPitchVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addSpeed(builder, speedOffset) {
    builder.addFieldOffset(9, speedOffset, 0);
  }
  static createSpeedVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startSpeedVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addYawMode(builder, yawMode) {
    builder.addFieldInt8(10, yawMode, 0);
  }
  static addTraceMode(builder, traceMode) {
    builder.addFieldInt8(11, traceMode, 0);
  }
  static addGimbalPitchMode(builder, gimbalPitchMode) {
    builder.addFieldInt8(12, gimbalPitchMode, 0);
  }
  static addFinishAction(builder, finishAction) {
    builder.addFieldInt8(13, finishAction, 0);
  }
  static addCommandList(builder, commandListOffset) {
    builder.addFieldOffset(14, commandListOffset, 0);
  }
  static createCommandListVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startCommandListVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addCommandParameter(builder, commandParameterOffset) {
    builder.addFieldOffset(15, commandParameterOffset, 0);
  }
  static createCommandParameterVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startCommandParameterVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static endConfigMissionRequest(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createConfigMissionRequest(builder, uavIdOffset, missionIdOffset, missionType, waypointOffset, radius, maxVel, idleVel, yawOffset, gimbalPitchOffset, speedOffset, yawMode, traceMode, gimbalPitchMode, finishAction, commandListOffset, commandParameterOffset) {
    _ConfigMissionRequest.startConfigMissionRequest(builder);
    _ConfigMissionRequest.addUavId(builder, uavIdOffset);
    _ConfigMissionRequest.addMissionId(builder, missionIdOffset);
    _ConfigMissionRequest.addMissionType(builder, missionType);
    _ConfigMissionRequest.addWaypoint(builder, waypointOffset);
    _ConfigMissionRequest.addRadius(builder, radius);
    _ConfigMissionRequest.addMaxVel(builder, maxVel);
    _ConfigMissionRequest.addIdleVel(builder, idleVel);
    _ConfigMissionRequest.addYaw(builder, yawOffset);
    _ConfigMissionRequest.addGimbalPitch(builder, gimbalPitchOffset);
    _ConfigMissionRequest.addSpeed(builder, speedOffset);
    _ConfigMissionRequest.addYawMode(builder, yawMode);
    _ConfigMissionRequest.addTraceMode(builder, traceMode);
    _ConfigMissionRequest.addGimbalPitchMode(builder, gimbalPitchMode);
    _ConfigMissionRequest.addFinishAction(builder, finishAction);
    _ConfigMissionRequest.addCommandList(builder, commandListOffset);
    _ConfigMissionRequest.addCommandParameter(builder, commandParameterOffset);
    return _ConfigMissionRequest.endConfigMissionRequest(builder);
  }
};

// fbmsglib/src/fb/aerialcore-common/config-mission-response.ts
var flatbuffers7 = __toESM(require("flatbuffers"));
var ConfigMissionResponse = class _ConfigMissionResponse {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsConfigMissionResponse(bb, obj) {
    return (obj || new _ConfigMissionResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsConfigMissionResponse(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers7.SIZE_PREFIX_LENGTH);
    return (obj || new _ConfigMissionResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  success() {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  static startConfigMissionResponse(builder) {
    builder.startObject(1);
  }
  static addSuccess(builder, success) {
    builder.addFieldInt8(0, +success, 0);
  }
  static endConfigMissionResponse(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createConfigMissionResponse(builder, success) {
    _ConfigMissionResponse.startConfigMissionResponse(builder);
    _ConfigMissionResponse.addSuccess(builder, success);
    return _ConfigMissionResponse.endConfigMissionResponse(builder);
  }
};

// fbmsglib/src/fb/aerialcore-common/config-mission.ts
var ConfigMission = class _ConfigMission {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsConfigMission(bb, obj) {
    return (obj || new _ConfigMission()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsConfigMission(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers8.SIZE_PREFIX_LENGTH);
    return (obj || new _ConfigMission()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  request(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new ConfigMissionRequest()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  response(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new ConfigMissionResponse()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startConfigMission(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addRequest(builder, requestOffset) {
    builder.addFieldOffset(1, requestOffset, 0);
  }
  static addResponse(builder, responseOffset) {
    builder.addFieldOffset(2, responseOffset, 0);
  }
  static endConfigMission(builder) {
    const offset = builder.endObject();
    return offset;
  }
};

// fbmsglib/src/fb/aerialcore-common/event-request.ts
var flatbuffers9 = __toESM(require("flatbuffers"));
var eventRequest = class _eventRequest {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAseventRequest(bb, obj) {
    return (obj || new _eventRequest()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAseventRequest(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers9.SIZE_PREFIX_LENGTH);
    return (obj || new _eventRequest()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  uavId(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  data(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  static starteventRequest(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addUavId(builder, uavIdOffset) {
    builder.addFieldOffset(1, uavIdOffset, 0);
  }
  static addData(builder, dataOffset) {
    builder.addFieldOffset(2, dataOffset, 0);
  }
  static endeventRequest(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createeventRequest(builder, _MetadataOffset, uavIdOffset, dataOffset) {
    _eventRequest.starteventRequest(builder);
    _eventRequest.add_Metadata(builder, _MetadataOffset);
    _eventRequest.addUavId(builder, uavIdOffset);
    _eventRequest.addData(builder, dataOffset);
    return _eventRequest.endeventRequest(builder);
  }
};

// fbmsglib/src/fb/aerialcore-common/event-response.ts
var flatbuffers10 = __toESM(require("flatbuffers"));
var eventResponse = class _eventResponse {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAseventResponse(bb, obj) {
    return (obj || new _eventResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAseventResponse(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers10.SIZE_PREFIX_LENGTH);
    return (obj || new _eventResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  success() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  msg(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  static starteventResponse(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addSuccess(builder, success) {
    builder.addFieldInt8(1, +success, 0);
  }
  static addMsg(builder, msgOffset) {
    builder.addFieldOffset(2, msgOffset, 0);
  }
  static endeventResponse(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createeventResponse(builder, _MetadataOffset, success, msgOffset) {
    _eventResponse.starteventResponse(builder);
    _eventResponse.add_Metadata(builder, _MetadataOffset);
    _eventResponse.addSuccess(builder, success);
    _eventResponse.addMsg(builder, msgOffset);
    return _eventResponse.endeventResponse(builder);
  }
};

// fbmsglib/src/fb/aerialcore-common/event-service.ts
var flatbuffers11 = __toESM(require("flatbuffers"));
var eventService = class _eventService {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAseventService(bb, obj) {
    return (obj || new _eventService()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAseventService(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers11.SIZE_PREFIX_LENGTH);
    return (obj || new _eventService()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  request(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new eventRequest()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  response(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new eventResponse()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static starteventService(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addRequest(builder, requestOffset) {
    builder.addFieldOffset(1, requestOffset, 0);
  }
  static addResponse(builder, responseOffset) {
    builder.addFieldOffset(2, responseOffset, 0);
  }
  static endeventService(builder) {
    const offset = builder.endObject();
    return offset;
  }
};

// fbmsglib/src/fb/amrl-msgs.ts
var amrl_msgs_exports = {};
__export(amrl_msgs_exports, {
  ColoredArc2D: () => ColoredArc2D,
  ColoredLine2D: () => ColoredLine2D,
  ColoredPoint2D: () => ColoredPoint2D,
  ElevatorCommand: () => ElevatorCommand,
  ElevatorStatus: () => ElevatorStatus,
  ErrorReport: () => ErrorReport,
  Localization2DMsg: () => Localization2DMsg,
  PathVisualization: () => PathVisualization,
  Point2D: () => Point2D,
  Pose2Df: () => Pose2Df,
  RobofleetStatus: () => RobofleetStatus,
  RobofleetSubscription: () => RobofleetSubscription,
  VisualizationMsg: () => VisualizationMsg
});

// fbmsglib/src/fb/amrl-msgs/colored-arc2-d.ts
var flatbuffers13 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/amrl-msgs/point2-d.ts
var flatbuffers12 = __toESM(require("flatbuffers"));
var Point2D = class _Point2D {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPoint2D(bb, obj) {
    return (obj || new _Point2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPoint2D(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers12.SIZE_PREFIX_LENGTH);
    return (obj || new _Point2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  x() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  y() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  static startPoint2D(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addX(builder, x) {
    builder.addFieldFloat32(1, x, 0);
  }
  static addY(builder, y) {
    builder.addFieldFloat32(2, y, 0);
  }
  static endPoint2D(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createPoint2D(builder, _MetadataOffset, x, y) {
    _Point2D.startPoint2D(builder);
    _Point2D.add_Metadata(builder, _MetadataOffset);
    _Point2D.addX(builder, x);
    _Point2D.addY(builder, y);
    return _Point2D.endPoint2D(builder);
  }
};

// fbmsglib/src/fb/amrl-msgs/colored-arc2-d.ts
var ColoredArc2D = class _ColoredArc2D {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsColoredArc2D(bb, obj) {
    return (obj || new _ColoredArc2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsColoredArc2D(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers13.SIZE_PREFIX_LENGTH);
    return (obj || new _ColoredArc2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  center(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Point2D()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  radius() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  startAngle() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  endAngle() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  color() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  static startColoredArc2D(builder) {
    builder.startObject(6);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addCenter(builder, centerOffset) {
    builder.addFieldOffset(1, centerOffset, 0);
  }
  static addRadius(builder, radius) {
    builder.addFieldFloat32(2, radius, 0);
  }
  static addStartAngle(builder, startAngle) {
    builder.addFieldFloat32(3, startAngle, 0);
  }
  static addEndAngle(builder, endAngle) {
    builder.addFieldFloat32(4, endAngle, 0);
  }
  static addColor(builder, color) {
    builder.addFieldInt32(5, color, 0);
  }
  static endColoredArc2D(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    return offset;
  }
};

// fbmsglib/src/fb/amrl-msgs/colored-line2-d.ts
var flatbuffers14 = __toESM(require("flatbuffers"));
var ColoredLine2D = class _ColoredLine2D {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsColoredLine2D(bb, obj) {
    return (obj || new _ColoredLine2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsColoredLine2D(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers14.SIZE_PREFIX_LENGTH);
    return (obj || new _ColoredLine2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  p0(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Point2D()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  p1(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new Point2D()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  color() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  static startColoredLine2D(builder) {
    builder.startObject(4);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addP0(builder, p0Offset) {
    builder.addFieldOffset(1, p0Offset, 0);
  }
  static addP1(builder, p1Offset) {
    builder.addFieldOffset(2, p1Offset, 0);
  }
  static addColor(builder, color) {
    builder.addFieldInt32(3, color, 0);
  }
  static endColoredLine2D(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    return offset;
  }
};

// fbmsglib/src/fb/amrl-msgs/colored-point2-d.ts
var flatbuffers15 = __toESM(require("flatbuffers"));
var ColoredPoint2D = class _ColoredPoint2D {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsColoredPoint2D(bb, obj) {
    return (obj || new _ColoredPoint2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsColoredPoint2D(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers15.SIZE_PREFIX_LENGTH);
    return (obj || new _ColoredPoint2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  point(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Point2D()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  color() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  static startColoredPoint2D(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addPoint(builder, pointOffset) {
    builder.addFieldOffset(1, pointOffset, 0);
  }
  static addColor(builder, color) {
    builder.addFieldInt32(2, color, 0);
  }
  static endColoredPoint2D(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    return offset;
  }
};

// fbmsglib/src/fb/amrl-msgs/elevator-command.ts
var flatbuffers16 = __toESM(require("flatbuffers"));
var ElevatorCommand = class _ElevatorCommand {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsElevatorCommand(bb, obj) {
    return (obj || new _ElevatorCommand()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsElevatorCommand(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers16.SIZE_PREFIX_LENGTH);
    return (obj || new _ElevatorCommand()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  floorCmd() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  holdDoor() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  static startElevatorCommand(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addFloorCmd(builder, floorCmd) {
    builder.addFieldInt8(1, floorCmd, 0);
  }
  static addHoldDoor(builder, holdDoor) {
    builder.addFieldInt8(2, +holdDoor, 0);
  }
  static endElevatorCommand(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createElevatorCommand(builder, _MetadataOffset, floorCmd, holdDoor) {
    _ElevatorCommand.startElevatorCommand(builder);
    _ElevatorCommand.add_Metadata(builder, _MetadataOffset);
    _ElevatorCommand.addFloorCmd(builder, floorCmd);
    _ElevatorCommand.addHoldDoor(builder, holdDoor);
    return _ElevatorCommand.endElevatorCommand(builder);
  }
};

// fbmsglib/src/fb/amrl-msgs/elevator-status.ts
var flatbuffers17 = __toESM(require("flatbuffers"));
var ElevatorStatus = class _ElevatorStatus {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsElevatorStatus(bb, obj) {
    return (obj || new _ElevatorStatus()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsElevatorStatus(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers17.SIZE_PREFIX_LENGTH);
    return (obj || new _ElevatorStatus()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  floor() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  door() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  static startElevatorStatus(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addFloor(builder, floor) {
    builder.addFieldInt8(1, floor, 0);
  }
  static addDoor(builder, door) {
    builder.addFieldInt8(2, door, 0);
  }
  static endElevatorStatus(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createElevatorStatus(builder, _MetadataOffset, floor, door) {
    _ElevatorStatus.startElevatorStatus(builder);
    _ElevatorStatus.add_Metadata(builder, _MetadataOffset);
    _ElevatorStatus.addFloor(builder, floor);
    _ElevatorStatus.addDoor(builder, door);
    return _ElevatorStatus.endElevatorStatus(builder);
  }
};

// fbmsglib/src/fb/amrl-msgs/error-report.ts
var flatbuffers18 = __toESM(require("flatbuffers"));
var ErrorReport = class _ErrorReport {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsErrorReport(bb, obj) {
    return (obj || new _ErrorReport()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsErrorReport(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers18.SIZE_PREFIX_LENGTH);
    return (obj || new _ErrorReport()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  laserHeader(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  severityLevel() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  failedSubsystem() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  detailedErrorMsg(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  static startErrorReport(builder) {
    builder.startObject(6);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addLaserHeader(builder, laserHeaderOffset) {
    builder.addFieldOffset(2, laserHeaderOffset, 0);
  }
  static addSeverityLevel(builder, severityLevel) {
    builder.addFieldInt8(3, severityLevel, 0);
  }
  static addFailedSubsystem(builder, failedSubsystem) {
    builder.addFieldInt8(4, failedSubsystem, 0);
  }
  static addDetailedErrorMsg(builder, detailedErrorMsgOffset) {
    builder.addFieldOffset(5, detailedErrorMsgOffset, 0);
  }
  static endErrorReport(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    builder.requiredField(offset, 14);
    return offset;
  }
};

// fbmsglib/src/fb/amrl-msgs/localization2-dmsg.ts
var flatbuffers20 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/amrl-msgs/pose2-df.ts
var flatbuffers19 = __toESM(require("flatbuffers"));
var Pose2Df = class _Pose2Df {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPose2Df(bb, obj) {
    return (obj || new _Pose2Df()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPose2Df(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers19.SIZE_PREFIX_LENGTH);
    return (obj || new _Pose2Df()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  x() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  y() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  theta() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  static startPose2Df(builder) {
    builder.startObject(4);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addX(builder, x) {
    builder.addFieldFloat32(1, x, 0);
  }
  static addY(builder, y) {
    builder.addFieldFloat32(2, y, 0);
  }
  static addTheta(builder, theta) {
    builder.addFieldFloat32(3, theta, 0);
  }
  static endPose2Df(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createPose2Df(builder, _MetadataOffset, x, y, theta) {
    _Pose2Df.startPose2Df(builder);
    _Pose2Df.add_Metadata(builder, _MetadataOffset);
    _Pose2Df.addX(builder, x);
    _Pose2Df.addY(builder, y);
    _Pose2Df.addTheta(builder, theta);
    return _Pose2Df.endPose2Df(builder);
  }
};

// fbmsglib/src/fb/amrl-msgs/localization2-dmsg.ts
var Localization2DMsg = class _Localization2DMsg {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsLocalization2DMsg(bb, obj) {
    return (obj || new _Localization2DMsg()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsLocalization2DMsg(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers20.SIZE_PREFIX_LENGTH);
    return (obj || new _Localization2DMsg()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  pose(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new Pose2Df()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  map(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  static startLocalization2DMsg(builder) {
    builder.startObject(4);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addPose(builder, poseOffset) {
    builder.addFieldOffset(2, poseOffset, 0);
  }
  static addMap(builder, mapOffset) {
    builder.addFieldOffset(3, mapOffset, 0);
  }
  static endLocalization2DMsg(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    builder.requiredField(offset, 10);
    return offset;
  }
};

// fbmsglib/src/fb/amrl-msgs/path-visualization.ts
var flatbuffers21 = __toESM(require("flatbuffers"));
var PathVisualization = class _PathVisualization {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPathVisualization(bb, obj) {
    return (obj || new _PathVisualization()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPathVisualization(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers21.SIZE_PREFIX_LENGTH);
    return (obj || new _PathVisualization()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  curvature() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  distance() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  clearance() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  static startPathVisualization(builder) {
    builder.startObject(4);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addCurvature(builder, curvature) {
    builder.addFieldFloat32(1, curvature, 0);
  }
  static addDistance(builder, distance) {
    builder.addFieldFloat32(2, distance, 0);
  }
  static addClearance(builder, clearance) {
    builder.addFieldFloat32(3, clearance, 0);
  }
  static endPathVisualization(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createPathVisualization(builder, _MetadataOffset, curvature, distance, clearance) {
    _PathVisualization.startPathVisualization(builder);
    _PathVisualization.add_Metadata(builder, _MetadataOffset);
    _PathVisualization.addCurvature(builder, curvature);
    _PathVisualization.addDistance(builder, distance);
    _PathVisualization.addClearance(builder, clearance);
    return _PathVisualization.endPathVisualization(builder);
  }
};

// fbmsglib/src/fb/amrl-msgs/robofleet-status.ts
var flatbuffers22 = __toESM(require("flatbuffers"));
var RobofleetStatus = class _RobofleetStatus {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsRobofleetStatus(bb, obj) {
    return (obj || new _RobofleetStatus()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsRobofleetStatus(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers22.SIZE_PREFIX_LENGTH);
    return (obj || new _RobofleetStatus()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  status(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  isOk() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  batteryLevel() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  location(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  static startRobofleetStatus(builder) {
    builder.startObject(5);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addStatus(builder, statusOffset) {
    builder.addFieldOffset(1, statusOffset, 0);
  }
  static addIsOk(builder, isOk) {
    builder.addFieldInt8(2, +isOk, 0);
  }
  static addBatteryLevel(builder, batteryLevel) {
    builder.addFieldFloat32(3, batteryLevel, 0);
  }
  static addLocation(builder, locationOffset) {
    builder.addFieldOffset(4, locationOffset, 0);
  }
  static endRobofleetStatus(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 12);
    return offset;
  }
  static createRobofleetStatus(builder, _MetadataOffset, statusOffset, isOk, batteryLevel, locationOffset) {
    _RobofleetStatus.startRobofleetStatus(builder);
    _RobofleetStatus.add_Metadata(builder, _MetadataOffset);
    _RobofleetStatus.addStatus(builder, statusOffset);
    _RobofleetStatus.addIsOk(builder, isOk);
    _RobofleetStatus.addBatteryLevel(builder, batteryLevel);
    _RobofleetStatus.addLocation(builder, locationOffset);
    return _RobofleetStatus.endRobofleetStatus(builder);
  }
};

// fbmsglib/src/fb/amrl-msgs/robofleet-subscription.ts
var flatbuffers23 = __toESM(require("flatbuffers"));
var RobofleetSubscription = class _RobofleetSubscription {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsRobofleetSubscription(bb, obj) {
    return (obj || new _RobofleetSubscription()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsRobofleetSubscription(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers23.SIZE_PREFIX_LENGTH);
    return (obj || new _RobofleetSubscription()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  topicRegex(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  action() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  static startRobofleetSubscription(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addTopicRegex(builder, topicRegexOffset) {
    builder.addFieldOffset(1, topicRegexOffset, 0);
  }
  static addAction(builder, action) {
    builder.addFieldInt8(2, action, 0);
  }
  static endRobofleetSubscription(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    return offset;
  }
  static createRobofleetSubscription(builder, _MetadataOffset, topicRegexOffset, action) {
    _RobofleetSubscription.startRobofleetSubscription(builder);
    _RobofleetSubscription.add_Metadata(builder, _MetadataOffset);
    _RobofleetSubscription.addTopicRegex(builder, topicRegexOffset);
    _RobofleetSubscription.addAction(builder, action);
    return _RobofleetSubscription.endRobofleetSubscription(builder);
  }
};

// fbmsglib/src/fb/amrl-msgs/visualization-msg.ts
var flatbuffers24 = __toESM(require("flatbuffers"));
var VisualizationMsg = class _VisualizationMsg {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsVisualizationMsg(bb, obj) {
    return (obj || new _VisualizationMsg()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsVisualizationMsg(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers24.SIZE_PREFIX_LENGTH);
    return (obj || new _VisualizationMsg()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  ns(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  particles(index, obj) {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? (obj || new Pose2Df()).__init(this.bb.__indirect(this.bb.__vector(this.bb_pos + offset) + index * 4), this.bb) : null;
  }
  particlesLength() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  pathOptions(index, obj) {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? (obj || new PathVisualization()).__init(this.bb.__indirect(this.bb.__vector(this.bb_pos + offset) + index * 4), this.bb) : null;
  }
  pathOptionsLength() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  points(index, obj) {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? (obj || new ColoredPoint2D()).__init(this.bb.__indirect(this.bb.__vector(this.bb_pos + offset) + index * 4), this.bb) : null;
  }
  pointsLength() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  lines(index, obj) {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? (obj || new ColoredLine2D()).__init(this.bb.__indirect(this.bb.__vector(this.bb_pos + offset) + index * 4), this.bb) : null;
  }
  linesLength() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  arcs(index, obj) {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? (obj || new ColoredArc2D()).__init(this.bb.__indirect(this.bb.__vector(this.bb_pos + offset) + index * 4), this.bb) : null;
  }
  arcsLength() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  static startVisualizationMsg(builder) {
    builder.startObject(8);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addNs(builder, nsOffset) {
    builder.addFieldOffset(2, nsOffset, 0);
  }
  static addParticles(builder, particlesOffset) {
    builder.addFieldOffset(3, particlesOffset, 0);
  }
  static createParticlesVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addOffset(data[i]);
    }
    return builder.endVector();
  }
  static startParticlesVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addPathOptions(builder, pathOptionsOffset) {
    builder.addFieldOffset(4, pathOptionsOffset, 0);
  }
  static createPathOptionsVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addOffset(data[i]);
    }
    return builder.endVector();
  }
  static startPathOptionsVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addPoints(builder, pointsOffset) {
    builder.addFieldOffset(5, pointsOffset, 0);
  }
  static createPointsVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addOffset(data[i]);
    }
    return builder.endVector();
  }
  static startPointsVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addLines(builder, linesOffset) {
    builder.addFieldOffset(6, linesOffset, 0);
  }
  static createLinesVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addOffset(data[i]);
    }
    return builder.endVector();
  }
  static startLinesVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addArcs(builder, arcsOffset) {
    builder.addFieldOffset(7, arcsOffset, 0);
  }
  static createArcsVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addOffset(data[i]);
    }
    return builder.endVector();
  }
  static startArcsVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static endVisualizationMsg(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    builder.requiredField(offset, 10);
    builder.requiredField(offset, 12);
    builder.requiredField(offset, 14);
    builder.requiredField(offset, 16);
    builder.requiredField(offset, 18);
    return offset;
  }
};

// fbmsglib/src/fb/dji-osdk-ros.ts
var dji_osdk_ros_exports = {};
__export(dji_osdk_ros_exports, {
  ObstacleInfo: () => ObstacleInfo,
  WaypointV2MissionStatePush: () => WaypointV2MissionStatePush
});

// fbmsglib/src/fb/dji-osdk-ros/obstacle-info.ts
var flatbuffers25 = __toESM(require("flatbuffers"));
var ObstacleInfo = class _ObstacleInfo {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsObstacleInfo(bb, obj) {
    return (obj || new _ObstacleInfo()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsObstacleInfo(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers25.SIZE_PREFIX_LENGTH);
    return (obj || new _ObstacleInfo()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  down() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  front() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  right() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  back() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  left() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  up() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  healtNotWorking() {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  healtWorking() {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 1;
  }
  downHealth() {
    const offset = this.bb.__offset(this.bb_pos, 24);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  frontHealth() {
    const offset = this.bb.__offset(this.bb_pos, 26);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  rightHealth() {
    const offset = this.bb.__offset(this.bb_pos, 28);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  backHealth() {
    const offset = this.bb.__offset(this.bb_pos, 30);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  leftHealth() {
    const offset = this.bb.__offset(this.bb_pos, 32);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  upHealth() {
    const offset = this.bb.__offset(this.bb_pos, 34);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  static startObstacleInfo(builder) {
    builder.startObject(16);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addDown(builder, down) {
    builder.addFieldFloat32(2, down, 0);
  }
  static addFront(builder, front) {
    builder.addFieldFloat32(3, front, 0);
  }
  static addRight(builder, right) {
    builder.addFieldFloat32(4, right, 0);
  }
  static addBack(builder, back) {
    builder.addFieldFloat32(5, back, 0);
  }
  static addLeft(builder, left) {
    builder.addFieldFloat32(6, left, 0);
  }
  static addUp(builder, up) {
    builder.addFieldFloat32(7, up, 0);
  }
  static addHealtNotWorking(builder, healtNotWorking) {
    builder.addFieldInt8(8, healtNotWorking, 0);
  }
  static addHealtWorking(builder, healtWorking) {
    builder.addFieldInt8(9, healtWorking, 1);
  }
  static addDownHealth(builder, downHealth) {
    builder.addFieldInt8(10, downHealth, 0);
  }
  static addFrontHealth(builder, frontHealth) {
    builder.addFieldInt8(11, frontHealth, 0);
  }
  static addRightHealth(builder, rightHealth) {
    builder.addFieldInt8(12, rightHealth, 0);
  }
  static addBackHealth(builder, backHealth) {
    builder.addFieldInt8(13, backHealth, 0);
  }
  static addLeftHealth(builder, leftHealth) {
    builder.addFieldInt8(14, leftHealth, 0);
  }
  static addUpHealth(builder, upHealth) {
    builder.addFieldInt8(15, upHealth, 0);
  }
  static endObstacleInfo(builder) {
    const offset = builder.endObject();
    return offset;
  }
};

// fbmsglib/src/fb/dji-osdk-ros/waypoint-v2-mission-state-push.ts
var flatbuffers26 = __toESM(require("flatbuffers"));
var WaypointV2MissionStatePush = class _WaypointV2MissionStatePush {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsWaypointV2MissionStatePush(bb, obj) {
    return (obj || new _WaypointV2MissionStatePush()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsWaypointV2MissionStatePush(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers26.SIZE_PREFIX_LENGTH);
    return (obj || new _WaypointV2MissionStatePush()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  commonDataVersion() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  commonDataLen() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint16(this.bb_pos + offset) : 0;
  }
  curWaypointIndex() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readUint16(this.bb_pos + offset) : 0;
  }
  state() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  velocity() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readUint16(this.bb_pos + offset) : 0;
  }
  static startWaypointV2MissionStatePush(builder) {
    builder.startObject(6);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addCommonDataVersion(builder, commonDataVersion) {
    builder.addFieldInt8(1, commonDataVersion, 0);
  }
  static addCommonDataLen(builder, commonDataLen) {
    builder.addFieldInt16(2, commonDataLen, 0);
  }
  static addCurWaypointIndex(builder, curWaypointIndex) {
    builder.addFieldInt16(3, curWaypointIndex, 0);
  }
  static addState(builder, state) {
    builder.addFieldInt8(4, state, 0);
  }
  static addVelocity(builder, velocity) {
    builder.addFieldInt16(5, velocity, 0);
  }
  static endWaypointV2MissionStatePush(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createWaypointV2MissionStatePush(builder, _MetadataOffset, commonDataVersion, commonDataLen, curWaypointIndex, state, velocity) {
    _WaypointV2MissionStatePush.startWaypointV2MissionStatePush(builder);
    _WaypointV2MissionStatePush.add_Metadata(builder, _MetadataOffset);
    _WaypointV2MissionStatePush.addCommonDataVersion(builder, commonDataVersion);
    _WaypointV2MissionStatePush.addCommonDataLen(builder, commonDataLen);
    _WaypointV2MissionStatePush.addCurWaypointIndex(builder, curWaypointIndex);
    _WaypointV2MissionStatePush.addState(builder, state);
    _WaypointV2MissionStatePush.addVelocity(builder, velocity);
    return _WaypointV2MissionStatePush.endWaypointV2MissionStatePush(builder);
  }
};

// fbmsglib/src/fb/geometry-msgs.ts
var geometry_msgs_exports = {};
__export(geometry_msgs_exports, {
  Point: () => Point,
  Pose: () => Pose,
  PoseStamped: () => PoseStamped,
  PoseWithCovariance: () => PoseWithCovariance,
  PoseWithCovarianceStamped: () => PoseWithCovarianceStamped,
  Quaternion: () => Quaternion,
  Twist: () => Twist,
  TwistStamped: () => TwistStamped,
  TwistWithCovariance: () => TwistWithCovariance,
  Vector3: () => Vector3,
  Vector3Stamped: () => Vector3Stamped
});

// fbmsglib/src/fb/geometry-msgs/point.ts
var flatbuffers27 = __toESM(require("flatbuffers"));
var Point = class _Point {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPoint(bb, obj) {
    return (obj || new _Point()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPoint(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers27.SIZE_PREFIX_LENGTH);
    return (obj || new _Point()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  x() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  y() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  z() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  static startPoint(builder) {
    builder.startObject(4);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addX(builder, x) {
    builder.addFieldFloat64(1, x, 0);
  }
  static addY(builder, y) {
    builder.addFieldFloat64(2, y, 0);
  }
  static addZ(builder, z) {
    builder.addFieldFloat64(3, z, 0);
  }
  static endPoint(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createPoint(builder, _MetadataOffset, x, y, z) {
    _Point.startPoint(builder);
    _Point.add_Metadata(builder, _MetadataOffset);
    _Point.addX(builder, x);
    _Point.addY(builder, y);
    _Point.addZ(builder, z);
    return _Point.endPoint(builder);
  }
};

// fbmsglib/src/fb/geometry-msgs/pose.ts
var flatbuffers29 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/geometry-msgs/quaternion.ts
var flatbuffers28 = __toESM(require("flatbuffers"));
var Quaternion = class _Quaternion {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsQuaternion(bb, obj) {
    return (obj || new _Quaternion()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsQuaternion(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers28.SIZE_PREFIX_LENGTH);
    return (obj || new _Quaternion()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  x() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  y() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  z() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  w() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  static startQuaternion(builder) {
    builder.startObject(5);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addX(builder, x) {
    builder.addFieldFloat64(1, x, 0);
  }
  static addY(builder, y) {
    builder.addFieldFloat64(2, y, 0);
  }
  static addZ(builder, z) {
    builder.addFieldFloat64(3, z, 0);
  }
  static addW(builder, w) {
    builder.addFieldFloat64(4, w, 0);
  }
  static endQuaternion(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createQuaternion(builder, _MetadataOffset, x, y, z, w) {
    _Quaternion.startQuaternion(builder);
    _Quaternion.add_Metadata(builder, _MetadataOffset);
    _Quaternion.addX(builder, x);
    _Quaternion.addY(builder, y);
    _Quaternion.addZ(builder, z);
    _Quaternion.addW(builder, w);
    return _Quaternion.endQuaternion(builder);
  }
};

// fbmsglib/src/fb/geometry-msgs/pose.ts
var Pose = class _Pose {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPose(bb, obj) {
    return (obj || new _Pose()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPose(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers29.SIZE_PREFIX_LENGTH);
    return (obj || new _Pose()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  position(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Point()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  orientation(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new Quaternion()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startPose(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addPosition(builder, positionOffset) {
    builder.addFieldOffset(1, positionOffset, 0);
  }
  static addOrientation(builder, orientationOffset) {
    builder.addFieldOffset(2, orientationOffset, 0);
  }
  static endPose(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    return offset;
  }
};

// fbmsglib/src/fb/geometry-msgs/pose-stamped.ts
var flatbuffers30 = __toESM(require("flatbuffers"));
var PoseStamped = class _PoseStamped {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPoseStamped(bb, obj) {
    return (obj || new _PoseStamped()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPoseStamped(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers30.SIZE_PREFIX_LENGTH);
    return (obj || new _PoseStamped()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  pose(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new Pose()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startPoseStamped(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addPose(builder, poseOffset) {
    builder.addFieldOffset(2, poseOffset, 0);
  }
  static endPoseStamped(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    return offset;
  }
};

// fbmsglib/src/fb/geometry-msgs/pose-with-covariance.ts
var flatbuffers31 = __toESM(require("flatbuffers"));
var PoseWithCovariance = class _PoseWithCovariance {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPoseWithCovariance(bb, obj) {
    return (obj || new _PoseWithCovariance()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPoseWithCovariance(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers31.SIZE_PREFIX_LENGTH);
    return (obj || new _PoseWithCovariance()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  pose(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Pose()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  covariance(index) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat64(this.bb.__vector(this.bb_pos + offset) + index * 8) : 0;
  }
  covarianceLength() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  covarianceArray() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? new Float64Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  static startPoseWithCovariance(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addPose(builder, poseOffset) {
    builder.addFieldOffset(1, poseOffset, 0);
  }
  static addCovariance(builder, covarianceOffset) {
    builder.addFieldOffset(2, covarianceOffset, 0);
  }
  static createCovarianceVector(builder, data) {
    builder.startVector(8, data.length, 8);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat64(data[i]);
    }
    return builder.endVector();
  }
  static startCovarianceVector(builder, numElems) {
    builder.startVector(8, numElems, 8);
  }
  static endPoseWithCovariance(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    return offset;
  }
};

// fbmsglib/src/fb/geometry-msgs/pose-with-covariance-stamped.ts
var flatbuffers32 = __toESM(require("flatbuffers"));
var PoseWithCovarianceStamped = class _PoseWithCovarianceStamped {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPoseWithCovarianceStamped(bb, obj) {
    return (obj || new _PoseWithCovarianceStamped()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPoseWithCovarianceStamped(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers32.SIZE_PREFIX_LENGTH);
    return (obj || new _PoseWithCovarianceStamped()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  pose(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new PoseWithCovariance()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startPoseWithCovarianceStamped(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addPose(builder, poseOffset) {
    builder.addFieldOffset(2, poseOffset, 0);
  }
  static endPoseWithCovarianceStamped(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    return offset;
  }
};

// fbmsglib/src/fb/geometry-msgs/twist.ts
var flatbuffers34 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/geometry-msgs/vector3.ts
var flatbuffers33 = __toESM(require("flatbuffers"));
var Vector3 = class _Vector3 {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsVector3(bb, obj) {
    return (obj || new _Vector3()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsVector3(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers33.SIZE_PREFIX_LENGTH);
    return (obj || new _Vector3()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  x() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  y() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  z() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  static startVector3(builder) {
    builder.startObject(4);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addX(builder, x) {
    builder.addFieldFloat64(1, x, 0);
  }
  static addY(builder, y) {
    builder.addFieldFloat64(2, y, 0);
  }
  static addZ(builder, z) {
    builder.addFieldFloat64(3, z, 0);
  }
  static endVector3(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createVector3(builder, _MetadataOffset, x, y, z) {
    _Vector3.startVector3(builder);
    _Vector3.add_Metadata(builder, _MetadataOffset);
    _Vector3.addX(builder, x);
    _Vector3.addY(builder, y);
    _Vector3.addZ(builder, z);
    return _Vector3.endVector3(builder);
  }
};

// fbmsglib/src/fb/geometry-msgs/twist.ts
var Twist = class _Twist {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsTwist(bb, obj) {
    return (obj || new _Twist()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsTwist(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers34.SIZE_PREFIX_LENGTH);
    return (obj || new _Twist()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  linear(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Vector3()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  angular(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new Vector3()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startTwist(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addLinear(builder, linearOffset) {
    builder.addFieldOffset(1, linearOffset, 0);
  }
  static addAngular(builder, angularOffset) {
    builder.addFieldOffset(2, angularOffset, 0);
  }
  static endTwist(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    return offset;
  }
};

// fbmsglib/src/fb/geometry-msgs/twist-stamped.ts
var flatbuffers35 = __toESM(require("flatbuffers"));
var TwistStamped = class _TwistStamped {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsTwistStamped(bb, obj) {
    return (obj || new _TwistStamped()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsTwistStamped(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers35.SIZE_PREFIX_LENGTH);
    return (obj || new _TwistStamped()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  twist(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new Twist()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startTwistStamped(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addTwist(builder, twistOffset) {
    builder.addFieldOffset(2, twistOffset, 0);
  }
  static endTwistStamped(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    return offset;
  }
};

// fbmsglib/src/fb/geometry-msgs/twist-with-covariance.ts
var flatbuffers36 = __toESM(require("flatbuffers"));
var TwistWithCovariance = class _TwistWithCovariance {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsTwistWithCovariance(bb, obj) {
    return (obj || new _TwistWithCovariance()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsTwistWithCovariance(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers36.SIZE_PREFIX_LENGTH);
    return (obj || new _TwistWithCovariance()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  twist(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Twist()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  covariance(index) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat64(this.bb.__vector(this.bb_pos + offset) + index * 8) : 0;
  }
  covarianceLength() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  covarianceArray() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? new Float64Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  static startTwistWithCovariance(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addTwist(builder, twistOffset) {
    builder.addFieldOffset(1, twistOffset, 0);
  }
  static addCovariance(builder, covarianceOffset) {
    builder.addFieldOffset(2, covarianceOffset, 0);
  }
  static createCovarianceVector(builder, data) {
    builder.startVector(8, data.length, 8);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat64(data[i]);
    }
    return builder.endVector();
  }
  static startCovarianceVector(builder, numElems) {
    builder.startVector(8, numElems, 8);
  }
  static endTwistWithCovariance(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    return offset;
  }
};

// fbmsglib/src/fb/geometry-msgs/vector3-stamped.ts
var flatbuffers37 = __toESM(require("flatbuffers"));
var Vector3Stamped = class _Vector3Stamped {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsVector3Stamped(bb, obj) {
    return (obj || new _Vector3Stamped()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsVector3Stamped(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers37.SIZE_PREFIX_LENGTH);
    return (obj || new _Vector3Stamped()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  vector(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new Vector3()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startVector3Stamped(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addVector(builder, vectorOffset) {
    builder.addFieldOffset(2, vectorOffset, 0);
  }
  static endVector3Stamped(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    return offset;
  }
};

// fbmsglib/src/fb/mavros-msgs.ts
var mavros_msgs_exports = {};
__export(mavros_msgs_exports, {
  Altitude: () => Altitude
});

// fbmsglib/src/fb/mavros-msgs/altitude.ts
var flatbuffers38 = __toESM(require("flatbuffers"));
var Altitude = class _Altitude {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsAltitude(bb, obj) {
    return (obj || new _Altitude()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsAltitude(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers38.SIZE_PREFIX_LENGTH);
    return (obj || new _Altitude()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  monotonic() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  amsl() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  local() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  relative() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  terrain() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  bottomClearance() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  static startAltitude(builder) {
    builder.startObject(7);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addMonotonic(builder, monotonic) {
    builder.addFieldFloat32(1, monotonic, 0);
  }
  static addAmsl(builder, amsl) {
    builder.addFieldFloat32(2, amsl, 0);
  }
  static addLocal(builder, local) {
    builder.addFieldFloat32(3, local, 0);
  }
  static addRelative(builder, relative) {
    builder.addFieldFloat32(4, relative, 0);
  }
  static addTerrain(builder, terrain) {
    builder.addFieldFloat32(5, terrain, 0);
  }
  static addBottomClearance(builder, bottomClearance) {
    builder.addFieldFloat32(6, bottomClearance, 0);
  }
  static endAltitude(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createAltitude(builder, _MetadataOffset, monotonic, amsl, local, relative, terrain, bottomClearance) {
    _Altitude.startAltitude(builder);
    _Altitude.add_Metadata(builder, _MetadataOffset);
    _Altitude.addMonotonic(builder, monotonic);
    _Altitude.addAmsl(builder, amsl);
    _Altitude.addLocal(builder, local);
    _Altitude.addRelative(builder, relative);
    _Altitude.addTerrain(builder, terrain);
    _Altitude.addBottomClearance(builder, bottomClearance);
    return _Altitude.endAltitude(builder);
  }
};

// fbmsglib/src/fb/nav-msgs.ts
var nav_msgs_exports = {};
__export(nav_msgs_exports, {
  Odometry: () => Odometry
});

// fbmsglib/src/fb/nav-msgs/odometry.ts
var flatbuffers39 = __toESM(require("flatbuffers"));
var Odometry = class _Odometry {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsOdometry(bb, obj) {
    return (obj || new _Odometry()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsOdometry(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers39.SIZE_PREFIX_LENGTH);
    return (obj || new _Odometry()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  childFrameId(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  pose(obj) {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? (obj || new PoseWithCovariance()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  twist(obj) {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? (obj || new TwistWithCovariance()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startOdometry(builder) {
    builder.startObject(5);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addChildFrameId(builder, childFrameIdOffset) {
    builder.addFieldOffset(2, childFrameIdOffset, 0);
  }
  static addPose(builder, poseOffset) {
    builder.addFieldOffset(3, poseOffset, 0);
  }
  static addTwist(builder, twistOffset) {
    builder.addFieldOffset(4, twistOffset, 0);
  }
  static endOdometry(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    builder.requiredField(offset, 10);
    builder.requiredField(offset, 12);
    return offset;
  }
};

// fbmsglib/src/fb/sensor-msgs.ts
var sensor_msgs_exports = {};
__export(sensor_msgs_exports, {
  BatteryState: () => BatteryState,
  CompressedImage: () => CompressedImage,
  Imu: () => Imu,
  LaserScan: () => LaserScan,
  NavSatFix: () => NavSatFix,
  NavSatStatus: () => NavSatStatus,
  PointCloud2: () => PointCloud2,
  PointField: () => PointField
});

// fbmsglib/src/fb/sensor-msgs/battery-state.ts
var flatbuffers40 = __toESM(require("flatbuffers"));
var BatteryState = class _BatteryState {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsBatteryState(bb, obj) {
    return (obj || new _BatteryState()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsBatteryState(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers40.SIZE_PREFIX_LENGTH);
    return (obj || new _BatteryState()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  voltage() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  current() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  charge() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  capacity() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  designCapacity() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  percentage() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  powerSupplyStatus() {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.readInt8(this.bb_pos + offset) : 0;
  }
  powerSupplyHealth() {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? this.bb.readInt8(this.bb_pos + offset) : 0;
  }
  powerSupplyTechnology() {
    const offset = this.bb.__offset(this.bb_pos, 24);
    return offset ? this.bb.readInt8(this.bb_pos + offset) : 0;
  }
  present() {
    const offset = this.bb.__offset(this.bb_pos, 26);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  cellVoltage(index) {
    const offset = this.bb.__offset(this.bb_pos, 28);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  cellVoltageLength() {
    const offset = this.bb.__offset(this.bb_pos, 28);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  cellVoltageArray() {
    const offset = this.bb.__offset(this.bb_pos, 28);
    return offset ? new Float32Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  cellTemperature(index) {
    const offset = this.bb.__offset(this.bb_pos, 30);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  cellTemperatureLength() {
    const offset = this.bb.__offset(this.bb_pos, 30);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  cellTemperatureArray() {
    const offset = this.bb.__offset(this.bb_pos, 30);
    return offset ? new Float32Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  static startBatteryState(builder) {
    builder.startObject(14);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addVoltage(builder, voltage) {
    builder.addFieldFloat32(2, voltage, 0);
  }
  static addCurrent(builder, current) {
    builder.addFieldFloat32(3, current, 0);
  }
  static addCharge(builder, charge) {
    builder.addFieldFloat32(4, charge, 0);
  }
  static addCapacity(builder, capacity) {
    builder.addFieldFloat32(5, capacity, 0);
  }
  static addDesignCapacity(builder, designCapacity) {
    builder.addFieldFloat32(6, designCapacity, 0);
  }
  static addPercentage(builder, percentage) {
    builder.addFieldFloat32(7, percentage, 0);
  }
  static addPowerSupplyStatus(builder, powerSupplyStatus) {
    builder.addFieldInt8(8, powerSupplyStatus, 0);
  }
  static addPowerSupplyHealth(builder, powerSupplyHealth) {
    builder.addFieldInt8(9, powerSupplyHealth, 0);
  }
  static addPowerSupplyTechnology(builder, powerSupplyTechnology) {
    builder.addFieldInt8(10, powerSupplyTechnology, 0);
  }
  static addPresent(builder, present) {
    builder.addFieldInt8(11, +present, 0);
  }
  static addCellVoltage(builder, cellVoltageOffset) {
    builder.addFieldOffset(12, cellVoltageOffset, 0);
  }
  static createCellVoltageVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startCellVoltageVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addCellTemperature(builder, cellTemperatureOffset) {
    builder.addFieldOffset(13, cellTemperatureOffset, 0);
  }
  static createCellTemperatureVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startCellTemperatureVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static endBatteryState(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 28);
    builder.requiredField(offset, 30);
    return offset;
  }
};

// fbmsglib/src/fb/sensor-msgs/compressed-image.ts
var flatbuffers41 = __toESM(require("flatbuffers"));
var CompressedImage = class _CompressedImage {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsCompressedImage(bb, obj) {
    return (obj || new _CompressedImage()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsCompressedImage(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers41.SIZE_PREFIX_LENGTH);
    return (obj || new _CompressedImage()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  format(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  data(index) {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readUint8(this.bb.__vector(this.bb_pos + offset) + index) : 0;
  }
  dataLength() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  dataArray() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? new Uint8Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  static startCompressedImage(builder) {
    builder.startObject(4);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addFormat(builder, formatOffset) {
    builder.addFieldOffset(2, formatOffset, 0);
  }
  static addData(builder, dataOffset) {
    builder.addFieldOffset(3, dataOffset, 0);
  }
  static createDataVector(builder, data) {
    builder.startVector(1, data.length, 1);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addInt8(data[i]);
    }
    return builder.endVector();
  }
  static startDataVector(builder, numElems) {
    builder.startVector(1, numElems, 1);
  }
  static endCompressedImage(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    builder.requiredField(offset, 10);
    return offset;
  }
};

// fbmsglib/src/fb/sensor-msgs/imu.ts
var flatbuffers42 = __toESM(require("flatbuffers"));
var Imu = class _Imu {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsImu(bb, obj) {
    return (obj || new _Imu()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsImu(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers42.SIZE_PREFIX_LENGTH);
    return (obj || new _Imu()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  orientation(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new Quaternion()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  orientationCovariance(index) {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat64(this.bb.__vector(this.bb_pos + offset) + index * 8) : 0;
  }
  orientationCovarianceLength() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  orientationCovarianceArray() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? new Float64Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  angularVelocity(obj) {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? (obj || new Vector3()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  angularVelocityCovariance(index) {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat64(this.bb.__vector(this.bb_pos + offset) + index * 8) : 0;
  }
  angularVelocityCovarianceLength() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  angularVelocityCovarianceArray() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? new Float64Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  linearAcceleration(obj) {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? (obj || new Vector3()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  linearAccelerationCovariance(index) {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readFloat64(this.bb.__vector(this.bb_pos + offset) + index * 8) : 0;
  }
  linearAccelerationCovarianceLength() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  linearAccelerationCovarianceArray() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? new Float64Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  static startImu(builder) {
    builder.startObject(8);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addOrientation(builder, orientationOffset) {
    builder.addFieldOffset(2, orientationOffset, 0);
  }
  static addOrientationCovariance(builder, orientationCovarianceOffset) {
    builder.addFieldOffset(3, orientationCovarianceOffset, 0);
  }
  static createOrientationCovarianceVector(builder, data) {
    builder.startVector(8, data.length, 8);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat64(data[i]);
    }
    return builder.endVector();
  }
  static startOrientationCovarianceVector(builder, numElems) {
    builder.startVector(8, numElems, 8);
  }
  static addAngularVelocity(builder, angularVelocityOffset) {
    builder.addFieldOffset(4, angularVelocityOffset, 0);
  }
  static addAngularVelocityCovariance(builder, angularVelocityCovarianceOffset) {
    builder.addFieldOffset(5, angularVelocityCovarianceOffset, 0);
  }
  static createAngularVelocityCovarianceVector(builder, data) {
    builder.startVector(8, data.length, 8);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat64(data[i]);
    }
    return builder.endVector();
  }
  static startAngularVelocityCovarianceVector(builder, numElems) {
    builder.startVector(8, numElems, 8);
  }
  static addLinearAcceleration(builder, linearAccelerationOffset) {
    builder.addFieldOffset(6, linearAccelerationOffset, 0);
  }
  static addLinearAccelerationCovariance(builder, linearAccelerationCovarianceOffset) {
    builder.addFieldOffset(7, linearAccelerationCovarianceOffset, 0);
  }
  static createLinearAccelerationCovarianceVector(builder, data) {
    builder.startVector(8, data.length, 8);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat64(data[i]);
    }
    return builder.endVector();
  }
  static startLinearAccelerationCovarianceVector(builder, numElems) {
    builder.startVector(8, numElems, 8);
  }
  static endImu(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 8);
    builder.requiredField(offset, 10);
    builder.requiredField(offset, 12);
    builder.requiredField(offset, 14);
    builder.requiredField(offset, 16);
    builder.requiredField(offset, 18);
    return offset;
  }
};

// fbmsglib/src/fb/sensor-msgs/laser-scan.ts
var flatbuffers43 = __toESM(require("flatbuffers"));
var LaserScan = class _LaserScan {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsLaserScan(bb, obj) {
    return (obj || new _LaserScan()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsLaserScan(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers43.SIZE_PREFIX_LENGTH);
    return (obj || new _LaserScan()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  angleMin() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  angleMax() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  angleIncrement() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  timeIncrement() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  scanTime() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  rangeMin() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  rangeMax() {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  ranges(index) {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  rangesLength() {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  rangesArray() {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? new Float32Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  intensities(index) {
    const offset = this.bb.__offset(this.bb_pos, 24);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  intensitiesLength() {
    const offset = this.bb.__offset(this.bb_pos, 24);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  intensitiesArray() {
    const offset = this.bb.__offset(this.bb_pos, 24);
    return offset ? new Float32Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  static startLaserScan(builder) {
    builder.startObject(11);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addAngleMin(builder, angleMin) {
    builder.addFieldFloat32(2, angleMin, 0);
  }
  static addAngleMax(builder, angleMax) {
    builder.addFieldFloat32(3, angleMax, 0);
  }
  static addAngleIncrement(builder, angleIncrement) {
    builder.addFieldFloat32(4, angleIncrement, 0);
  }
  static addTimeIncrement(builder, timeIncrement) {
    builder.addFieldFloat32(5, timeIncrement, 0);
  }
  static addScanTime(builder, scanTime) {
    builder.addFieldFloat32(6, scanTime, 0);
  }
  static addRangeMin(builder, rangeMin) {
    builder.addFieldFloat32(7, rangeMin, 0);
  }
  static addRangeMax(builder, rangeMax) {
    builder.addFieldFloat32(8, rangeMax, 0);
  }
  static addRanges(builder, rangesOffset) {
    builder.addFieldOffset(9, rangesOffset, 0);
  }
  static createRangesVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startRangesVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addIntensities(builder, intensitiesOffset) {
    builder.addFieldOffset(10, intensitiesOffset, 0);
  }
  static createIntensitiesVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startIntensitiesVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static endLaserScan(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 22);
    builder.requiredField(offset, 24);
    return offset;
  }
};

// fbmsglib/src/fb/sensor-msgs/point-cloud2.ts
var flatbuffers45 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/sensor-msgs/point-field.ts
var flatbuffers44 = __toESM(require("flatbuffers"));
var PointField = class _PointField {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPointField(bb, obj) {
    return (obj || new _PointField()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPointField(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers44.SIZE_PREFIX_LENGTH);
    return (obj || new _PointField()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  name(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  offset() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  datatype() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  count() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  static startPointField(builder) {
    builder.startObject(5);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addName(builder, nameOffset) {
    builder.addFieldOffset(1, nameOffset, 0);
  }
  static addOffset(builder, offset) {
    builder.addFieldInt32(2, offset, 0);
  }
  static addDatatype(builder, datatype) {
    builder.addFieldInt8(3, datatype, 0);
  }
  static addCount(builder, count) {
    builder.addFieldInt32(4, count, 0);
  }
  static endPointField(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    return offset;
  }
  static createPointField(builder, _MetadataOffset, nameOffset, offset, datatype, count) {
    _PointField.startPointField(builder);
    _PointField.add_Metadata(builder, _MetadataOffset);
    _PointField.addName(builder, nameOffset);
    _PointField.addOffset(builder, offset);
    _PointField.addDatatype(builder, datatype);
    _PointField.addCount(builder, count);
    return _PointField.endPointField(builder);
  }
};

// fbmsglib/src/fb/sensor-msgs/point-cloud2.ts
var PointCloud2 = class _PointCloud2 {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPointCloud2(bb, obj) {
    return (obj || new _PointCloud2()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPointCloud2(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers45.SIZE_PREFIX_LENGTH);
    return (obj || new _PointCloud2()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  height() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  width() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  fields(index, obj) {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? (obj || new PointField()).__init(this.bb.__indirect(this.bb.__vector(this.bb_pos + offset) + index * 4), this.bb) : null;
  }
  fieldsLength() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  isBigendian() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  pointStep() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  rowStep() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  data(index) {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.readUint8(this.bb.__vector(this.bb_pos + offset) + index) : 0;
  }
  dataLength() {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  dataArray() {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? new Uint8Array(this.bb.bytes().buffer, this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset), this.bb.__vector_len(this.bb_pos + offset)) : null;
  }
  isDense() {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  static startPointCloud2(builder) {
    builder.startObject(10);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addHeight(builder, height) {
    builder.addFieldInt32(2, height, 0);
  }
  static addWidth(builder, width) {
    builder.addFieldInt32(3, width, 0);
  }
  static addFields(builder, fieldsOffset) {
    builder.addFieldOffset(4, fieldsOffset, 0);
  }
  static createFieldsVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addOffset(data[i]);
    }
    return builder.endVector();
  }
  static startFieldsVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addIsBigendian(builder, isBigendian) {
    builder.addFieldInt8(5, +isBigendian, 0);
  }
  static addPointStep(builder, pointStep) {
    builder.addFieldInt32(6, pointStep, 0);
  }
  static addRowStep(builder, rowStep) {
    builder.addFieldInt32(7, rowStep, 0);
  }
  static addData(builder, dataOffset) {
    builder.addFieldOffset(8, dataOffset, 0);
  }
  static createDataVector(builder, data) {
    builder.startVector(1, data.length, 1);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addInt8(data[i]);
    }
    return builder.endVector();
  }
  static startDataVector(builder, numElems) {
    builder.startVector(1, numElems, 1);
  }
  static addIsDense(builder, isDense) {
    builder.addFieldInt8(9, +isDense, 0);
  }
  static endPointCloud2(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    builder.requiredField(offset, 12);
    builder.requiredField(offset, 20);
    return offset;
  }
};

// fbmsglib/src/fb/std-msgs.ts
var std_msgs_exports = {};
__export(std_msgs_exports, {
  Float32: () => Float32,
  Float64: () => Float64,
  Header: () => Header,
  String: () => String,
  UInt8: () => UInt8
});

// fbmsglib/src/fb/std-msgs/float32.ts
var flatbuffers46 = __toESM(require("flatbuffers"));
var Float32 = class _Float32 {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsFloat32(bb, obj) {
    return (obj || new _Float32()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsFloat32(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers46.SIZE_PREFIX_LENGTH);
    return (obj || new _Float32()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  data() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0;
  }
  static startFloat32(builder) {
    builder.startObject(2);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addData(builder, data) {
    builder.addFieldFloat32(1, data, 0);
  }
  static endFloat32(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createFloat32(builder, _MetadataOffset, data) {
    _Float32.startFloat32(builder);
    _Float32.add_Metadata(builder, _MetadataOffset);
    _Float32.addData(builder, data);
    return _Float32.endFloat32(builder);
  }
};

// fbmsglib/src/fb/std-msgs/float64.ts
var flatbuffers47 = __toESM(require("flatbuffers"));
var Float64 = class _Float64 {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsFloat64(bb, obj) {
    return (obj || new _Float64()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsFloat64(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers47.SIZE_PREFIX_LENGTH);
    return (obj || new _Float64()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  data() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0;
  }
  static startFloat64(builder) {
    builder.startObject(2);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addData(builder, data) {
    builder.addFieldFloat64(1, data, 0);
  }
  static endFloat64(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createFloat64(builder, _MetadataOffset, data) {
    _Float64.startFloat64(builder);
    _Float64.add_Metadata(builder, _MetadataOffset);
    _Float64.addData(builder, data);
    return _Float64.endFloat64(builder);
  }
};

// fbmsglib/src/fb/std-msgs/string.ts
var flatbuffers48 = __toESM(require("flatbuffers"));
var String = class _String {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsString(bb, obj) {
    return (obj || new _String()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsString(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers48.SIZE_PREFIX_LENGTH);
    return (obj || new _String()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  data(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  static startString(builder) {
    builder.startObject(2);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addData(builder, dataOffset) {
    builder.addFieldOffset(1, dataOffset, 0);
  }
  static endString(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6);
    return offset;
  }
  static createString(builder, _MetadataOffset, dataOffset) {
    _String.startString(builder);
    _String.add_Metadata(builder, _MetadataOffset);
    _String.addData(builder, dataOffset);
    return _String.endString(builder);
  }
};

// fbmsglib/src/fb/std-msgs/uint8.ts
var flatbuffers49 = __toESM(require("flatbuffers"));
var UInt8 = class _UInt8 {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsUInt8(bb, obj) {
    return (obj || new _UInt8()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsUInt8(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers49.SIZE_PREFIX_LENGTH);
    return (obj || new _UInt8()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  data() {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  static startUInt8(builder) {
    builder.startObject(2);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addData(builder, data) {
    builder.addFieldInt8(1, data, 0);
  }
  static endUInt8(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createUInt8(builder, _MetadataOffset, data) {
    _UInt8.startUInt8(builder);
    _UInt8.add_Metadata(builder, _MetadataOffset);
    _UInt8.addData(builder, data);
    return _UInt8.endUInt8(builder);
  }
};

// fbmsglib/src/fb/std-srvs.ts
var std_srvs_exports = {};
__export(std_srvs_exports, {
  Empty: () => Empty,
  EmptyRequest: () => EmptyRequest,
  EmptyResponse: () => EmptyResponse,
  SetBool: () => SetBool,
  SetBoolRequest: () => SetBoolRequest,
  SetBoolResponse: () => SetBoolResponse,
  Trigger: () => Trigger,
  TriggerRequest: () => TriggerRequest,
  TriggerResponse: () => TriggerResponse
});

// fbmsglib/src/fb/std-srvs/empty.ts
var flatbuffers52 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/std-srvs/empty-request.ts
var flatbuffers50 = __toESM(require("flatbuffers"));
var EmptyRequest = class _EmptyRequest {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsEmptyRequest(bb, obj) {
    return (obj || new _EmptyRequest()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsEmptyRequest(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers50.SIZE_PREFIX_LENGTH);
    return (obj || new _EmptyRequest()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static startEmptyRequest(builder) {
    builder.startObject(0);
  }
  static endEmptyRequest(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createEmptyRequest(builder) {
    _EmptyRequest.startEmptyRequest(builder);
    return _EmptyRequest.endEmptyRequest(builder);
  }
};

// fbmsglib/src/fb/std-srvs/empty-response.ts
var flatbuffers51 = __toESM(require("flatbuffers"));
var EmptyResponse = class _EmptyResponse {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsEmptyResponse(bb, obj) {
    return (obj || new _EmptyResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsEmptyResponse(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers51.SIZE_PREFIX_LENGTH);
    return (obj || new _EmptyResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static startEmptyResponse(builder) {
    builder.startObject(0);
  }
  static endEmptyResponse(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createEmptyResponse(builder) {
    _EmptyResponse.startEmptyResponse(builder);
    return _EmptyResponse.endEmptyResponse(builder);
  }
};

// fbmsglib/src/fb/std-srvs/empty.ts
var Empty = class _Empty {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsEmpty(bb, obj) {
    return (obj || new _Empty()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsEmpty(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers52.SIZE_PREFIX_LENGTH);
    return (obj || new _Empty()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  request(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new EmptyRequest()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  response(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new EmptyResponse()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startEmpty(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addRequest(builder, requestOffset) {
    builder.addFieldOffset(1, requestOffset, 0);
  }
  static addResponse(builder, responseOffset) {
    builder.addFieldOffset(2, responseOffset, 0);
  }
  static endEmpty(builder) {
    const offset = builder.endObject();
    return offset;
  }
};

// fbmsglib/src/fb/std-srvs/set-bool.ts
var flatbuffers55 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/std-srvs/set-bool-request.ts
var flatbuffers53 = __toESM(require("flatbuffers"));
var SetBoolRequest = class _SetBoolRequest {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsSetBoolRequest(bb, obj) {
    return (obj || new _SetBoolRequest()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsSetBoolRequest(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers53.SIZE_PREFIX_LENGTH);
    return (obj || new _SetBoolRequest()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  data() {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  static startSetBoolRequest(builder) {
    builder.startObject(1);
  }
  static addData(builder, data) {
    builder.addFieldInt8(0, +data, 0);
  }
  static endSetBoolRequest(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createSetBoolRequest(builder, data) {
    _SetBoolRequest.startSetBoolRequest(builder);
    _SetBoolRequest.addData(builder, data);
    return _SetBoolRequest.endSetBoolRequest(builder);
  }
};

// fbmsglib/src/fb/std-srvs/set-bool-response.ts
var flatbuffers54 = __toESM(require("flatbuffers"));
var SetBoolResponse = class _SetBoolResponse {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsSetBoolResponse(bb, obj) {
    return (obj || new _SetBoolResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsSetBoolResponse(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers54.SIZE_PREFIX_LENGTH);
    return (obj || new _SetBoolResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  success() {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  message(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  static startSetBoolResponse(builder) {
    builder.startObject(2);
  }
  static addSuccess(builder, success) {
    builder.addFieldInt8(0, +success, 0);
  }
  static addMessage(builder, messageOffset) {
    builder.addFieldOffset(1, messageOffset, 0);
  }
  static endSetBoolResponse(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createSetBoolResponse(builder, success, messageOffset) {
    _SetBoolResponse.startSetBoolResponse(builder);
    _SetBoolResponse.addSuccess(builder, success);
    _SetBoolResponse.addMessage(builder, messageOffset);
    return _SetBoolResponse.endSetBoolResponse(builder);
  }
};

// fbmsglib/src/fb/std-srvs/set-bool.ts
var SetBool = class _SetBool {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsSetBool(bb, obj) {
    return (obj || new _SetBool()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsSetBool(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers55.SIZE_PREFIX_LENGTH);
    return (obj || new _SetBool()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  request(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new SetBoolRequest()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  response(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new SetBoolResponse()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startSetBool(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addRequest(builder, requestOffset) {
    builder.addFieldOffset(1, requestOffset, 0);
  }
  static addResponse(builder, responseOffset) {
    builder.addFieldOffset(2, responseOffset, 0);
  }
  static endSetBool(builder) {
    const offset = builder.endObject();
    return offset;
  }
};

// fbmsglib/src/fb/std-srvs/trigger.ts
var flatbuffers58 = __toESM(require("flatbuffers"));

// fbmsglib/src/fb/std-srvs/trigger-request.ts
var flatbuffers56 = __toESM(require("flatbuffers"));
var TriggerRequest = class _TriggerRequest {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsTriggerRequest(bb, obj) {
    return (obj || new _TriggerRequest()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsTriggerRequest(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers56.SIZE_PREFIX_LENGTH);
    return (obj || new _TriggerRequest()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static startTriggerRequest(builder) {
    builder.startObject(0);
  }
  static endTriggerRequest(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createTriggerRequest(builder) {
    _TriggerRequest.startTriggerRequest(builder);
    return _TriggerRequest.endTriggerRequest(builder);
  }
};

// fbmsglib/src/fb/std-srvs/trigger-response.ts
var flatbuffers57 = __toESM(require("flatbuffers"));
var TriggerResponse = class _TriggerResponse {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsTriggerResponse(bb, obj) {
    return (obj || new _TriggerResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsTriggerResponse(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers57.SIZE_PREFIX_LENGTH);
    return (obj || new _TriggerResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  success() {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  message(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  static startTriggerResponse(builder) {
    builder.startObject(2);
  }
  static addSuccess(builder, success) {
    builder.addFieldInt8(0, +success, 0);
  }
  static addMessage(builder, messageOffset) {
    builder.addFieldOffset(1, messageOffset, 0);
  }
  static endTriggerResponse(builder) {
    const offset = builder.endObject();
    return offset;
  }
  static createTriggerResponse(builder, success, messageOffset) {
    _TriggerResponse.startTriggerResponse(builder);
    _TriggerResponse.addSuccess(builder, success);
    _TriggerResponse.addMessage(builder, messageOffset);
    return _TriggerResponse.endTriggerResponse(builder);
  }
};

// fbmsglib/src/fb/std-srvs/trigger.ts
var Trigger = class _Trigger {
  bb = null;
  bb_pos = 0;
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsTrigger(bb, obj) {
    return (obj || new _Trigger()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsTrigger(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers58.SIZE_PREFIX_LENGTH);
    return (obj || new _Trigger()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset ? (obj || new MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  request(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new TriggerRequest()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  response(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? (obj || new TriggerResponse()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  static startTrigger(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addRequest(builder, requestOffset) {
    builder.addFieldOffset(1, requestOffset, 0);
  }
  static addResponse(builder, responseOffset) {
    builder.addFieldOffset(2, responseOffset, 0);
  }
  static endTrigger(builder) {
    const offset = builder.endObject();
    return offset;
  }
};
// Annotate the CommonJS export names for ESM import in node:
0 && (module.exports = {
  fb
});
