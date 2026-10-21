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
  MsgMetadataT: () => MsgMetadataT,
  MsgWithMetadata: () => MsgWithMetadata,
  MsgWithMetadataT: () => MsgWithMetadataT,
  RosDuration: () => RosDuration,
  RosDurationT: () => RosDurationT,
  RosTime: () => RosTime,
  RosTimeT: () => RosTimeT,
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
  unpack() {
    return new MsgMetadataT(
      this.type(),
      this.topic()
    );
  }
  unpackTo(_o) {
    _o.type = this.type();
    _o.topic = this.topic();
  }
};
var MsgMetadataT = class {
  constructor(type = null, topic = null) {
    this.type = type;
    this.topic = topic;
  }
  pack(builder) {
    const type = this.type !== null ? builder.createString(this.type) : 0;
    const topic = this.topic !== null ? builder.createString(this.topic) : 0;
    return MsgMetadata.createMsgMetadata(
      builder,
      type,
      topic
    );
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
  unpack() {
    return new MsgWithMetadataT(
      this._Metadata() !== null ? this._Metadata().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
  }
};
var MsgWithMetadataT = class {
  constructor(_Metadata = null) {
    this._Metadata = _Metadata;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return MsgWithMetadata.createMsgWithMetadata(
      builder,
      _Metadata
    );
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
  unpack() {
    return new RosDurationT(
      this.secs(),
      this.nsecs()
    );
  }
  unpackTo(_o) {
    _o.secs = this.secs();
    _o.nsecs = this.nsecs();
  }
};
var RosDurationT = class {
  constructor(secs = 0, nsecs = 0) {
    this.secs = secs;
    this.nsecs = nsecs;
  }
  pack(builder) {
    return RosDuration.createRosDuration(
      builder,
      this.secs,
      this.nsecs
    );
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
  unpack() {
    return new RosTimeT(
      this.secs(),
      this.nsecs()
    );
  }
  unpackTo(_o) {
    _o.secs = this.secs();
    _o.nsecs = this.nsecs();
  }
};
var RosTimeT = class {
  constructor(secs = 0, nsecs = 0) {
    this.secs = secs;
    this.nsecs = nsecs;
  }
  pack(builder) {
    return RosTime.createRosTime(
      builder,
      this.secs,
      this.nsecs
    );
  }
};

// fbmsglib/src/fb/aerialcore-common.ts
var aerialcore_common_exports = {};
__export(aerialcore_common_exports, {
  ConfigMission: () => ConfigMission,
  ConfigMissionRequest: () => ConfigMissionRequest,
  ConfigMissionRequestT: () => ConfigMissionRequestT,
  ConfigMissionResponse: () => ConfigMissionResponse,
  ConfigMissionResponseT: () => ConfigMissionResponseT,
  ConfigMissionT: () => ConfigMissionT,
  eventRequest: () => eventRequest,
  eventRequestT: () => eventRequestT,
  eventResponse: () => eventResponse,
  eventResponseT: () => eventResponseT,
  eventService: () => eventService,
  eventServiceT: () => eventServiceT
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
  unpack() {
    return new NavSatStatusT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.status(),
      this.service()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.status = this.status();
    _o.service = this.service();
  }
};
var NavSatStatusT = class {
  constructor(_Metadata = null, status = 0, service = 0) {
    this._Metadata = _Metadata;
    this.status = status;
    this.service = service;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return NavSatStatus.createNavSatStatus(
      builder,
      _Metadata,
      this.status,
      this.service
    );
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
  unpack() {
    return new HeaderT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.seq(),
      this.stamp() !== null ? this.stamp().unpack() : null,
      this.frameId()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.seq = this.seq();
    _o.stamp = this.stamp() !== null ? this.stamp().unpack() : null;
    _o.frameId = this.frameId();
  }
};
var HeaderT = class {
  constructor(_Metadata = null, seq = 0, stamp = null, frameId = null) {
    this._Metadata = _Metadata;
    this.seq = seq;
    this.stamp = stamp;
    this.frameId = frameId;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const frameId = this.frameId !== null ? builder.createString(this.frameId) : 0;
    Header.startHeader(builder);
    Header.add_Metadata(builder, _Metadata);
    Header.addSeq(builder, this.seq);
    Header.addStamp(builder, this.stamp !== null ? this.stamp.pack(builder) : 0);
    Header.addFrameId(builder, frameId);
    return Header.endHeader(builder);
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
  unpack() {
    return new NavSatFixT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.status() !== null ? this.status().unpack() : null,
      this.latitude(),
      this.longitude(),
      this.altitude(),
      this.bb.createScalarList(this.positionCovariance.bind(this), this.positionCovarianceLength()),
      this.positionCovarianceType()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.status = this.status() !== null ? this.status().unpack() : null;
    _o.latitude = this.latitude();
    _o.longitude = this.longitude();
    _o.altitude = this.altitude();
    _o.positionCovariance = this.bb.createScalarList(this.positionCovariance.bind(this), this.positionCovarianceLength());
    _o.positionCovarianceType = this.positionCovarianceType();
  }
};
var NavSatFixT = class {
  constructor(_Metadata = null, header = null, status = null, latitude = 0, longitude = 0, altitude = 0, positionCovariance = [], positionCovarianceType = 0) {
    this._Metadata = _Metadata;
    this.header = header;
    this.status = status;
    this.latitude = latitude;
    this.longitude = longitude;
    this.altitude = altitude;
    this.positionCovariance = positionCovariance;
    this.positionCovarianceType = positionCovarianceType;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const status = this.status !== null ? this.status.pack(builder) : 0;
    const positionCovariance = NavSatFix.createPositionCovarianceVector(builder, this.positionCovariance);
    NavSatFix.startNavSatFix(builder);
    NavSatFix.add_Metadata(builder, _Metadata);
    NavSatFix.addHeader(builder, header);
    NavSatFix.addStatus(builder, status);
    NavSatFix.addLatitude(builder, this.latitude);
    NavSatFix.addLongitude(builder, this.longitude);
    NavSatFix.addAltitude(builder, this.altitude);
    NavSatFix.addPositionCovariance(builder, positionCovariance);
    NavSatFix.addPositionCovarianceType(builder, this.positionCovarianceType);
    return NavSatFix.endNavSatFix(builder);
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
  unpack() {
    return new ConfigMissionRequestT(
      this.uavId(),
      this.missionId(),
      this.missionType(),
      this.bb.createObjList(this.waypoint.bind(this), this.waypointLength()),
      this.radius(),
      this.maxVel(),
      this.idleVel(),
      this.bb.createScalarList(this.yaw.bind(this), this.yawLength()),
      this.bb.createScalarList(this.gimbalPitch.bind(this), this.gimbalPitchLength()),
      this.bb.createScalarList(this.speed.bind(this), this.speedLength()),
      this.yawMode(),
      this.traceMode(),
      this.gimbalPitchMode(),
      this.finishAction(),
      this.bb.createScalarList(this.commandList.bind(this), this.commandListLength()),
      this.bb.createScalarList(this.commandParameter.bind(this), this.commandParameterLength())
    );
  }
  unpackTo(_o) {
    _o.uavId = this.uavId();
    _o.missionId = this.missionId();
    _o.missionType = this.missionType();
    _o.waypoint = this.bb.createObjList(this.waypoint.bind(this), this.waypointLength());
    _o.radius = this.radius();
    _o.maxVel = this.maxVel();
    _o.idleVel = this.idleVel();
    _o.yaw = this.bb.createScalarList(this.yaw.bind(this), this.yawLength());
    _o.gimbalPitch = this.bb.createScalarList(this.gimbalPitch.bind(this), this.gimbalPitchLength());
    _o.speed = this.bb.createScalarList(this.speed.bind(this), this.speedLength());
    _o.yawMode = this.yawMode();
    _o.traceMode = this.traceMode();
    _o.gimbalPitchMode = this.gimbalPitchMode();
    _o.finishAction = this.finishAction();
    _o.commandList = this.bb.createScalarList(this.commandList.bind(this), this.commandListLength());
    _o.commandParameter = this.bb.createScalarList(this.commandParameter.bind(this), this.commandParameterLength());
  }
};
var ConfigMissionRequestT = class {
  constructor(uavId = null, missionId = null, missionType = 0, waypoint = [], radius = 0, maxVel = 0, idleVel = 0, yaw = [], gimbalPitch = [], speed = [], yawMode = 0, traceMode = 0, gimbalPitchMode = 0, finishAction = 0, commandList = [], commandParameter = []) {
    this.uavId = uavId;
    this.missionId = missionId;
    this.missionType = missionType;
    this.waypoint = waypoint;
    this.radius = radius;
    this.maxVel = maxVel;
    this.idleVel = idleVel;
    this.yaw = yaw;
    this.gimbalPitch = gimbalPitch;
    this.speed = speed;
    this.yawMode = yawMode;
    this.traceMode = traceMode;
    this.gimbalPitchMode = gimbalPitchMode;
    this.finishAction = finishAction;
    this.commandList = commandList;
    this.commandParameter = commandParameter;
  }
  pack(builder) {
    const uavId = this.uavId !== null ? builder.createString(this.uavId) : 0;
    const missionId = this.missionId !== null ? builder.createString(this.missionId) : 0;
    const waypoint = ConfigMissionRequest.createWaypointVector(builder, builder.createObjectOffsetList(this.waypoint));
    const yaw = ConfigMissionRequest.createYawVector(builder, this.yaw);
    const gimbalPitch = ConfigMissionRequest.createGimbalPitchVector(builder, this.gimbalPitch);
    const speed = ConfigMissionRequest.createSpeedVector(builder, this.speed);
    const commandList = ConfigMissionRequest.createCommandListVector(builder, this.commandList);
    const commandParameter = ConfigMissionRequest.createCommandParameterVector(builder, this.commandParameter);
    return ConfigMissionRequest.createConfigMissionRequest(
      builder,
      uavId,
      missionId,
      this.missionType,
      waypoint,
      this.radius,
      this.maxVel,
      this.idleVel,
      yaw,
      gimbalPitch,
      speed,
      this.yawMode,
      this.traceMode,
      this.gimbalPitchMode,
      this.finishAction,
      commandList,
      commandParameter
    );
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
  unpack() {
    return new ConfigMissionResponseT(
      this.success()
    );
  }
  unpackTo(_o) {
    _o.success = this.success();
  }
};
var ConfigMissionResponseT = class {
  constructor(success = false) {
    this.success = success;
  }
  pack(builder) {
    return ConfigMissionResponse.createConfigMissionResponse(
      builder,
      this.success
    );
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
  unpack() {
    return new ConfigMissionT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.request() !== null ? this.request().unpack() : null,
      this.response() !== null ? this.response().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.request = this.request() !== null ? this.request().unpack() : null;
    _o.response = this.response() !== null ? this.response().unpack() : null;
  }
};
var ConfigMissionT = class {
  constructor(_Metadata = null, request = null, response = null) {
    this._Metadata = _Metadata;
    this.request = request;
    this.response = response;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const request = this.request !== null ? this.request.pack(builder) : 0;
    const response = this.response !== null ? this.response.pack(builder) : 0;
    ConfigMission.startConfigMission(builder);
    ConfigMission.add_Metadata(builder, _Metadata);
    ConfigMission.addRequest(builder, request);
    ConfigMission.addResponse(builder, response);
    return ConfigMission.endConfigMission(builder);
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
  unpack() {
    return new eventRequestT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.uavId(),
      this.data()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.uavId = this.uavId();
    _o.data = this.data();
  }
};
var eventRequestT = class {
  constructor(_Metadata = null, uavId = null, data = null) {
    this._Metadata = _Metadata;
    this.uavId = uavId;
    this.data = data;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const uavId = this.uavId !== null ? builder.createString(this.uavId) : 0;
    const data = this.data !== null ? builder.createString(this.data) : 0;
    return eventRequest.createeventRequest(
      builder,
      _Metadata,
      uavId,
      data
    );
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
  unpack() {
    return new eventResponseT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.success(),
      this.msg()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.success = this.success();
    _o.msg = this.msg();
  }
};
var eventResponseT = class {
  constructor(_Metadata = null, success = false, msg = null) {
    this._Metadata = _Metadata;
    this.success = success;
    this.msg = msg;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const msg = this.msg !== null ? builder.createString(this.msg) : 0;
    return eventResponse.createeventResponse(
      builder,
      _Metadata,
      this.success,
      msg
    );
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
  unpack() {
    return new eventServiceT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.request() !== null ? this.request().unpack() : null,
      this.response() !== null ? this.response().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.request = this.request() !== null ? this.request().unpack() : null;
    _o.response = this.response() !== null ? this.response().unpack() : null;
  }
};
var eventServiceT = class {
  constructor(_Metadata = null, request = null, response = null) {
    this._Metadata = _Metadata;
    this.request = request;
    this.response = response;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const request = this.request !== null ? this.request.pack(builder) : 0;
    const response = this.response !== null ? this.response.pack(builder) : 0;
    eventService.starteventService(builder);
    eventService.add_Metadata(builder, _Metadata);
    eventService.addRequest(builder, request);
    eventService.addResponse(builder, response);
    return eventService.endeventService(builder);
  }
};

// fbmsglib/src/fb/amrl-msgs.ts
var amrl_msgs_exports = {};
__export(amrl_msgs_exports, {
  ColoredArc2D: () => ColoredArc2D,
  ColoredArc2DT: () => ColoredArc2DT,
  ColoredLine2D: () => ColoredLine2D,
  ColoredLine2DT: () => ColoredLine2DT,
  ColoredPoint2D: () => ColoredPoint2D,
  ColoredPoint2DT: () => ColoredPoint2DT,
  ElevatorCommand: () => ElevatorCommand,
  ElevatorCommandT: () => ElevatorCommandT,
  ElevatorStatus: () => ElevatorStatus,
  ElevatorStatusT: () => ElevatorStatusT,
  ErrorReport: () => ErrorReport,
  ErrorReportT: () => ErrorReportT,
  Localization2DMsg: () => Localization2DMsg,
  Localization2DMsgT: () => Localization2DMsgT,
  PathVisualization: () => PathVisualization,
  PathVisualizationT: () => PathVisualizationT,
  Point2D: () => Point2D,
  Point2DT: () => Point2DT,
  Pose2Df: () => Pose2Df,
  Pose2DfT: () => Pose2DfT,
  RobofleetStatus: () => RobofleetStatus,
  RobofleetStatusT: () => RobofleetStatusT,
  RobofleetSubscription: () => RobofleetSubscription,
  RobofleetSubscriptionT: () => RobofleetSubscriptionT,
  VisualizationMsg: () => VisualizationMsg,
  VisualizationMsgT: () => VisualizationMsgT
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
  unpack() {
    return new Point2DT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.x(),
      this.y()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.x = this.x();
    _o.y = this.y();
  }
};
var Point2DT = class {
  constructor(_Metadata = null, x = 0, y = 0) {
    this._Metadata = _Metadata;
    this.x = x;
    this.y = y;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return Point2D.createPoint2D(
      builder,
      _Metadata,
      this.x,
      this.y
    );
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
  unpack() {
    return new ColoredArc2DT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.center() !== null ? this.center().unpack() : null,
      this.radius(),
      this.startAngle(),
      this.endAngle(),
      this.color()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.center = this.center() !== null ? this.center().unpack() : null;
    _o.radius = this.radius();
    _o.startAngle = this.startAngle();
    _o.endAngle = this.endAngle();
    _o.color = this.color();
  }
};
var ColoredArc2DT = class {
  constructor(_Metadata = null, center = null, radius = 0, startAngle = 0, endAngle = 0, color = 0) {
    this._Metadata = _Metadata;
    this.center = center;
    this.radius = radius;
    this.startAngle = startAngle;
    this.endAngle = endAngle;
    this.color = color;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const center = this.center !== null ? this.center.pack(builder) : 0;
    ColoredArc2D.startColoredArc2D(builder);
    ColoredArc2D.add_Metadata(builder, _Metadata);
    ColoredArc2D.addCenter(builder, center);
    ColoredArc2D.addRadius(builder, this.radius);
    ColoredArc2D.addStartAngle(builder, this.startAngle);
    ColoredArc2D.addEndAngle(builder, this.endAngle);
    ColoredArc2D.addColor(builder, this.color);
    return ColoredArc2D.endColoredArc2D(builder);
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
  unpack() {
    return new ColoredLine2DT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.p0() !== null ? this.p0().unpack() : null,
      this.p1() !== null ? this.p1().unpack() : null,
      this.color()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.p0 = this.p0() !== null ? this.p0().unpack() : null;
    _o.p1 = this.p1() !== null ? this.p1().unpack() : null;
    _o.color = this.color();
  }
};
var ColoredLine2DT = class {
  constructor(_Metadata = null, p0 = null, p1 = null, color = 0) {
    this._Metadata = _Metadata;
    this.p0 = p0;
    this.p1 = p1;
    this.color = color;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const p0 = this.p0 !== null ? this.p0.pack(builder) : 0;
    const p1 = this.p1 !== null ? this.p1.pack(builder) : 0;
    ColoredLine2D.startColoredLine2D(builder);
    ColoredLine2D.add_Metadata(builder, _Metadata);
    ColoredLine2D.addP0(builder, p0);
    ColoredLine2D.addP1(builder, p1);
    ColoredLine2D.addColor(builder, this.color);
    return ColoredLine2D.endColoredLine2D(builder);
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
  unpack() {
    return new ColoredPoint2DT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.point() !== null ? this.point().unpack() : null,
      this.color()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.point = this.point() !== null ? this.point().unpack() : null;
    _o.color = this.color();
  }
};
var ColoredPoint2DT = class {
  constructor(_Metadata = null, point = null, color = 0) {
    this._Metadata = _Metadata;
    this.point = point;
    this.color = color;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const point = this.point !== null ? this.point.pack(builder) : 0;
    ColoredPoint2D.startColoredPoint2D(builder);
    ColoredPoint2D.add_Metadata(builder, _Metadata);
    ColoredPoint2D.addPoint(builder, point);
    ColoredPoint2D.addColor(builder, this.color);
    return ColoredPoint2D.endColoredPoint2D(builder);
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
  unpack() {
    return new ElevatorCommandT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.floorCmd(),
      this.holdDoor()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.floorCmd = this.floorCmd();
    _o.holdDoor = this.holdDoor();
  }
};
var ElevatorCommandT = class {
  constructor(_Metadata = null, floorCmd = 0, holdDoor = false) {
    this._Metadata = _Metadata;
    this.floorCmd = floorCmd;
    this.holdDoor = holdDoor;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return ElevatorCommand.createElevatorCommand(
      builder,
      _Metadata,
      this.floorCmd,
      this.holdDoor
    );
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
  unpack() {
    return new ElevatorStatusT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.floor(),
      this.door()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.floor = this.floor();
    _o.door = this.door();
  }
};
var ElevatorStatusT = class {
  constructor(_Metadata = null, floor = 0, door = 0) {
    this._Metadata = _Metadata;
    this.floor = floor;
    this.door = door;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return ElevatorStatus.createElevatorStatus(
      builder,
      _Metadata,
      this.floor,
      this.door
    );
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
  unpack() {
    return new ErrorReportT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.laserHeader() !== null ? this.laserHeader().unpack() : null,
      this.severityLevel(),
      this.failedSubsystem(),
      this.detailedErrorMsg()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.laserHeader = this.laserHeader() !== null ? this.laserHeader().unpack() : null;
    _o.severityLevel = this.severityLevel();
    _o.failedSubsystem = this.failedSubsystem();
    _o.detailedErrorMsg = this.detailedErrorMsg();
  }
};
var ErrorReportT = class {
  constructor(_Metadata = null, header = null, laserHeader = null, severityLevel = 0, failedSubsystem = 0, detailedErrorMsg = null) {
    this._Metadata = _Metadata;
    this.header = header;
    this.laserHeader = laserHeader;
    this.severityLevel = severityLevel;
    this.failedSubsystem = failedSubsystem;
    this.detailedErrorMsg = detailedErrorMsg;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const laserHeader = this.laserHeader !== null ? this.laserHeader.pack(builder) : 0;
    const detailedErrorMsg = this.detailedErrorMsg !== null ? builder.createString(this.detailedErrorMsg) : 0;
    ErrorReport.startErrorReport(builder);
    ErrorReport.add_Metadata(builder, _Metadata);
    ErrorReport.addHeader(builder, header);
    ErrorReport.addLaserHeader(builder, laserHeader);
    ErrorReport.addSeverityLevel(builder, this.severityLevel);
    ErrorReport.addFailedSubsystem(builder, this.failedSubsystem);
    ErrorReport.addDetailedErrorMsg(builder, detailedErrorMsg);
    return ErrorReport.endErrorReport(builder);
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
  unpack() {
    return new Pose2DfT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.x(),
      this.y(),
      this.theta()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.x = this.x();
    _o.y = this.y();
    _o.theta = this.theta();
  }
};
var Pose2DfT = class {
  constructor(_Metadata = null, x = 0, y = 0, theta = 0) {
    this._Metadata = _Metadata;
    this.x = x;
    this.y = y;
    this.theta = theta;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return Pose2Df.createPose2Df(
      builder,
      _Metadata,
      this.x,
      this.y,
      this.theta
    );
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
  unpack() {
    return new Localization2DMsgT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.pose() !== null ? this.pose().unpack() : null,
      this.map()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.pose = this.pose() !== null ? this.pose().unpack() : null;
    _o.map = this.map();
  }
};
var Localization2DMsgT = class {
  constructor(_Metadata = null, header = null, pose = null, map = null) {
    this._Metadata = _Metadata;
    this.header = header;
    this.pose = pose;
    this.map = map;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const pose = this.pose !== null ? this.pose.pack(builder) : 0;
    const map = this.map !== null ? builder.createString(this.map) : 0;
    Localization2DMsg.startLocalization2DMsg(builder);
    Localization2DMsg.add_Metadata(builder, _Metadata);
    Localization2DMsg.addHeader(builder, header);
    Localization2DMsg.addPose(builder, pose);
    Localization2DMsg.addMap(builder, map);
    return Localization2DMsg.endLocalization2DMsg(builder);
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
  unpack() {
    return new PathVisualizationT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.curvature(),
      this.distance(),
      this.clearance()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.curvature = this.curvature();
    _o.distance = this.distance();
    _o.clearance = this.clearance();
  }
};
var PathVisualizationT = class {
  constructor(_Metadata = null, curvature = 0, distance = 0, clearance = 0) {
    this._Metadata = _Metadata;
    this.curvature = curvature;
    this.distance = distance;
    this.clearance = clearance;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return PathVisualization.createPathVisualization(
      builder,
      _Metadata,
      this.curvature,
      this.distance,
      this.clearance
    );
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
  unpack() {
    return new RobofleetStatusT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.status(),
      this.isOk(),
      this.batteryLevel(),
      this.location()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.status = this.status();
    _o.isOk = this.isOk();
    _o.batteryLevel = this.batteryLevel();
    _o.location = this.location();
  }
};
var RobofleetStatusT = class {
  constructor(_Metadata = null, status = null, isOk = false, batteryLevel = 0, location = null) {
    this._Metadata = _Metadata;
    this.status = status;
    this.isOk = isOk;
    this.batteryLevel = batteryLevel;
    this.location = location;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const status = this.status !== null ? builder.createString(this.status) : 0;
    const location = this.location !== null ? builder.createString(this.location) : 0;
    return RobofleetStatus.createRobofleetStatus(
      builder,
      _Metadata,
      status,
      this.isOk,
      this.batteryLevel,
      location
    );
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
  unpack() {
    return new RobofleetSubscriptionT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.topicRegex(),
      this.action()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.topicRegex = this.topicRegex();
    _o.action = this.action();
  }
};
var RobofleetSubscriptionT = class {
  constructor(_Metadata = null, topicRegex = null, action = 0) {
    this._Metadata = _Metadata;
    this.topicRegex = topicRegex;
    this.action = action;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const topicRegex = this.topicRegex !== null ? builder.createString(this.topicRegex) : 0;
    return RobofleetSubscription.createRobofleetSubscription(
      builder,
      _Metadata,
      topicRegex,
      this.action
    );
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
  unpack() {
    return new VisualizationMsgT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.ns(),
      this.bb.createObjList(this.particles.bind(this), this.particlesLength()),
      this.bb.createObjList(this.pathOptions.bind(this), this.pathOptionsLength()),
      this.bb.createObjList(this.points.bind(this), this.pointsLength()),
      this.bb.createObjList(this.lines.bind(this), this.linesLength()),
      this.bb.createObjList(this.arcs.bind(this), this.arcsLength())
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.ns = this.ns();
    _o.particles = this.bb.createObjList(this.particles.bind(this), this.particlesLength());
    _o.pathOptions = this.bb.createObjList(this.pathOptions.bind(this), this.pathOptionsLength());
    _o.points = this.bb.createObjList(this.points.bind(this), this.pointsLength());
    _o.lines = this.bb.createObjList(this.lines.bind(this), this.linesLength());
    _o.arcs = this.bb.createObjList(this.arcs.bind(this), this.arcsLength());
  }
};
var VisualizationMsgT = class {
  constructor(_Metadata = null, header = null, ns = null, particles = [], pathOptions = [], points = [], lines = [], arcs = []) {
    this._Metadata = _Metadata;
    this.header = header;
    this.ns = ns;
    this.particles = particles;
    this.pathOptions = pathOptions;
    this.points = points;
    this.lines = lines;
    this.arcs = arcs;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const ns = this.ns !== null ? builder.createString(this.ns) : 0;
    const particles = VisualizationMsg.createParticlesVector(builder, builder.createObjectOffsetList(this.particles));
    const pathOptions = VisualizationMsg.createPathOptionsVector(builder, builder.createObjectOffsetList(this.pathOptions));
    const points = VisualizationMsg.createPointsVector(builder, builder.createObjectOffsetList(this.points));
    const lines = VisualizationMsg.createLinesVector(builder, builder.createObjectOffsetList(this.lines));
    const arcs = VisualizationMsg.createArcsVector(builder, builder.createObjectOffsetList(this.arcs));
    VisualizationMsg.startVisualizationMsg(builder);
    VisualizationMsg.add_Metadata(builder, _Metadata);
    VisualizationMsg.addHeader(builder, header);
    VisualizationMsg.addNs(builder, ns);
    VisualizationMsg.addParticles(builder, particles);
    VisualizationMsg.addPathOptions(builder, pathOptions);
    VisualizationMsg.addPoints(builder, points);
    VisualizationMsg.addLines(builder, lines);
    VisualizationMsg.addArcs(builder, arcs);
    return VisualizationMsg.endVisualizationMsg(builder);
  }
};

// fbmsglib/src/fb/dji-osdk-ros.ts
var dji_osdk_ros_exports = {};
__export(dji_osdk_ros_exports, {
  ObstacleInfo: () => ObstacleInfo,
  ObstacleInfoT: () => ObstacleInfoT,
  WaypointV2MissionStatePush: () => WaypointV2MissionStatePush,
  WaypointV2MissionStatePushT: () => WaypointV2MissionStatePushT
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
  unpack() {
    return new ObstacleInfoT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.down(),
      this.front(),
      this.right(),
      this.back(),
      this.left(),
      this.up(),
      this.healtNotWorking(),
      this.healtWorking(),
      this.downHealth(),
      this.frontHealth(),
      this.rightHealth(),
      this.backHealth(),
      this.leftHealth(),
      this.upHealth()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.down = this.down();
    _o.front = this.front();
    _o.right = this.right();
    _o.back = this.back();
    _o.left = this.left();
    _o.up = this.up();
    _o.healtNotWorking = this.healtNotWorking();
    _o.healtWorking = this.healtWorking();
    _o.downHealth = this.downHealth();
    _o.frontHealth = this.frontHealth();
    _o.rightHealth = this.rightHealth();
    _o.backHealth = this.backHealth();
    _o.leftHealth = this.leftHealth();
    _o.upHealth = this.upHealth();
  }
};
var ObstacleInfoT = class {
  constructor(_Metadata = null, header = null, down = 0, front = 0, right = 0, back = 0, left = 0, up = 0, healtNotWorking = 0, healtWorking = 1, downHealth = 0, frontHealth = 0, rightHealth = 0, backHealth = 0, leftHealth = 0, upHealth = 0) {
    this._Metadata = _Metadata;
    this.header = header;
    this.down = down;
    this.front = front;
    this.right = right;
    this.back = back;
    this.left = left;
    this.up = up;
    this.healtNotWorking = healtNotWorking;
    this.healtWorking = healtWorking;
    this.downHealth = downHealth;
    this.frontHealth = frontHealth;
    this.rightHealth = rightHealth;
    this.backHealth = backHealth;
    this.leftHealth = leftHealth;
    this.upHealth = upHealth;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    ObstacleInfo.startObstacleInfo(builder);
    ObstacleInfo.add_Metadata(builder, _Metadata);
    ObstacleInfo.addHeader(builder, header);
    ObstacleInfo.addDown(builder, this.down);
    ObstacleInfo.addFront(builder, this.front);
    ObstacleInfo.addRight(builder, this.right);
    ObstacleInfo.addBack(builder, this.back);
    ObstacleInfo.addLeft(builder, this.left);
    ObstacleInfo.addUp(builder, this.up);
    ObstacleInfo.addHealtNotWorking(builder, this.healtNotWorking);
    ObstacleInfo.addHealtWorking(builder, this.healtWorking);
    ObstacleInfo.addDownHealth(builder, this.downHealth);
    ObstacleInfo.addFrontHealth(builder, this.frontHealth);
    ObstacleInfo.addRightHealth(builder, this.rightHealth);
    ObstacleInfo.addBackHealth(builder, this.backHealth);
    ObstacleInfo.addLeftHealth(builder, this.leftHealth);
    ObstacleInfo.addUpHealth(builder, this.upHealth);
    return ObstacleInfo.endObstacleInfo(builder);
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
  unpack() {
    return new WaypointV2MissionStatePushT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.commonDataVersion(),
      this.commonDataLen(),
      this.curWaypointIndex(),
      this.state(),
      this.velocity()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.commonDataVersion = this.commonDataVersion();
    _o.commonDataLen = this.commonDataLen();
    _o.curWaypointIndex = this.curWaypointIndex();
    _o.state = this.state();
    _o.velocity = this.velocity();
  }
};
var WaypointV2MissionStatePushT = class {
  constructor(_Metadata = null, commonDataVersion = 0, commonDataLen = 0, curWaypointIndex = 0, state = 0, velocity = 0) {
    this._Metadata = _Metadata;
    this.commonDataVersion = commonDataVersion;
    this.commonDataLen = commonDataLen;
    this.curWaypointIndex = curWaypointIndex;
    this.state = state;
    this.velocity = velocity;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return WaypointV2MissionStatePush.createWaypointV2MissionStatePush(
      builder,
      _Metadata,
      this.commonDataVersion,
      this.commonDataLen,
      this.curWaypointIndex,
      this.state,
      this.velocity
    );
  }
};

// fbmsglib/src/fb/geometry-msgs.ts
var geometry_msgs_exports = {};
__export(geometry_msgs_exports, {
  Point: () => Point,
  PointT: () => PointT,
  Pose: () => Pose,
  PoseStamped: () => PoseStamped,
  PoseStampedT: () => PoseStampedT,
  PoseT: () => PoseT,
  PoseWithCovariance: () => PoseWithCovariance,
  PoseWithCovarianceStamped: () => PoseWithCovarianceStamped,
  PoseWithCovarianceStampedT: () => PoseWithCovarianceStampedT,
  PoseWithCovarianceT: () => PoseWithCovarianceT,
  Quaternion: () => Quaternion,
  QuaternionT: () => QuaternionT,
  Twist: () => Twist,
  TwistStamped: () => TwistStamped,
  TwistStampedT: () => TwistStampedT,
  TwistT: () => TwistT,
  TwistWithCovariance: () => TwistWithCovariance,
  TwistWithCovarianceT: () => TwistWithCovarianceT,
  Vector3: () => Vector3,
  Vector3Stamped: () => Vector3Stamped,
  Vector3StampedT: () => Vector3StampedT,
  Vector3T: () => Vector3T
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
  unpack() {
    return new PointT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.x(),
      this.y(),
      this.z()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.x = this.x();
    _o.y = this.y();
    _o.z = this.z();
  }
};
var PointT = class {
  constructor(_Metadata = null, x = 0, y = 0, z = 0) {
    this._Metadata = _Metadata;
    this.x = x;
    this.y = y;
    this.z = z;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return Point.createPoint(
      builder,
      _Metadata,
      this.x,
      this.y,
      this.z
    );
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
  unpack() {
    return new QuaternionT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.x(),
      this.y(),
      this.z(),
      this.w()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.x = this.x();
    _o.y = this.y();
    _o.z = this.z();
    _o.w = this.w();
  }
};
var QuaternionT = class {
  constructor(_Metadata = null, x = 0, y = 0, z = 0, w = 0) {
    this._Metadata = _Metadata;
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return Quaternion.createQuaternion(
      builder,
      _Metadata,
      this.x,
      this.y,
      this.z,
      this.w
    );
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
  unpack() {
    return new PoseT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.position() !== null ? this.position().unpack() : null,
      this.orientation() !== null ? this.orientation().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.position = this.position() !== null ? this.position().unpack() : null;
    _o.orientation = this.orientation() !== null ? this.orientation().unpack() : null;
  }
};
var PoseT = class {
  constructor(_Metadata = null, position = null, orientation = null) {
    this._Metadata = _Metadata;
    this.position = position;
    this.orientation = orientation;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const position = this.position !== null ? this.position.pack(builder) : 0;
    const orientation = this.orientation !== null ? this.orientation.pack(builder) : 0;
    Pose.startPose(builder);
    Pose.add_Metadata(builder, _Metadata);
    Pose.addPosition(builder, position);
    Pose.addOrientation(builder, orientation);
    return Pose.endPose(builder);
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
  unpack() {
    return new PoseStampedT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.pose() !== null ? this.pose().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.pose = this.pose() !== null ? this.pose().unpack() : null;
  }
};
var PoseStampedT = class {
  constructor(_Metadata = null, header = null, pose = null) {
    this._Metadata = _Metadata;
    this.header = header;
    this.pose = pose;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const pose = this.pose !== null ? this.pose.pack(builder) : 0;
    PoseStamped.startPoseStamped(builder);
    PoseStamped.add_Metadata(builder, _Metadata);
    PoseStamped.addHeader(builder, header);
    PoseStamped.addPose(builder, pose);
    return PoseStamped.endPoseStamped(builder);
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
  unpack() {
    return new PoseWithCovarianceT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.pose() !== null ? this.pose().unpack() : null,
      this.bb.createScalarList(this.covariance.bind(this), this.covarianceLength())
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.pose = this.pose() !== null ? this.pose().unpack() : null;
    _o.covariance = this.bb.createScalarList(this.covariance.bind(this), this.covarianceLength());
  }
};
var PoseWithCovarianceT = class {
  constructor(_Metadata = null, pose = null, covariance = []) {
    this._Metadata = _Metadata;
    this.pose = pose;
    this.covariance = covariance;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const pose = this.pose !== null ? this.pose.pack(builder) : 0;
    const covariance = PoseWithCovariance.createCovarianceVector(builder, this.covariance);
    PoseWithCovariance.startPoseWithCovariance(builder);
    PoseWithCovariance.add_Metadata(builder, _Metadata);
    PoseWithCovariance.addPose(builder, pose);
    PoseWithCovariance.addCovariance(builder, covariance);
    return PoseWithCovariance.endPoseWithCovariance(builder);
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
  unpack() {
    return new PoseWithCovarianceStampedT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.pose() !== null ? this.pose().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.pose = this.pose() !== null ? this.pose().unpack() : null;
  }
};
var PoseWithCovarianceStampedT = class {
  constructor(_Metadata = null, header = null, pose = null) {
    this._Metadata = _Metadata;
    this.header = header;
    this.pose = pose;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const pose = this.pose !== null ? this.pose.pack(builder) : 0;
    PoseWithCovarianceStamped.startPoseWithCovarianceStamped(builder);
    PoseWithCovarianceStamped.add_Metadata(builder, _Metadata);
    PoseWithCovarianceStamped.addHeader(builder, header);
    PoseWithCovarianceStamped.addPose(builder, pose);
    return PoseWithCovarianceStamped.endPoseWithCovarianceStamped(builder);
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
  unpack() {
    return new Vector3T(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.x(),
      this.y(),
      this.z()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.x = this.x();
    _o.y = this.y();
    _o.z = this.z();
  }
};
var Vector3T = class {
  constructor(_Metadata = null, x = 0, y = 0, z = 0) {
    this._Metadata = _Metadata;
    this.x = x;
    this.y = y;
    this.z = z;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return Vector3.createVector3(
      builder,
      _Metadata,
      this.x,
      this.y,
      this.z
    );
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
  unpack() {
    return new TwistT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.linear() !== null ? this.linear().unpack() : null,
      this.angular() !== null ? this.angular().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.linear = this.linear() !== null ? this.linear().unpack() : null;
    _o.angular = this.angular() !== null ? this.angular().unpack() : null;
  }
};
var TwistT = class {
  constructor(_Metadata = null, linear = null, angular = null) {
    this._Metadata = _Metadata;
    this.linear = linear;
    this.angular = angular;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const linear = this.linear !== null ? this.linear.pack(builder) : 0;
    const angular = this.angular !== null ? this.angular.pack(builder) : 0;
    Twist.startTwist(builder);
    Twist.add_Metadata(builder, _Metadata);
    Twist.addLinear(builder, linear);
    Twist.addAngular(builder, angular);
    return Twist.endTwist(builder);
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
  unpack() {
    return new TwistStampedT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.twist() !== null ? this.twist().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.twist = this.twist() !== null ? this.twist().unpack() : null;
  }
};
var TwistStampedT = class {
  constructor(_Metadata = null, header = null, twist = null) {
    this._Metadata = _Metadata;
    this.header = header;
    this.twist = twist;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const twist = this.twist !== null ? this.twist.pack(builder) : 0;
    TwistStamped.startTwistStamped(builder);
    TwistStamped.add_Metadata(builder, _Metadata);
    TwistStamped.addHeader(builder, header);
    TwistStamped.addTwist(builder, twist);
    return TwistStamped.endTwistStamped(builder);
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
  unpack() {
    return new TwistWithCovarianceT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.twist() !== null ? this.twist().unpack() : null,
      this.bb.createScalarList(this.covariance.bind(this), this.covarianceLength())
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.twist = this.twist() !== null ? this.twist().unpack() : null;
    _o.covariance = this.bb.createScalarList(this.covariance.bind(this), this.covarianceLength());
  }
};
var TwistWithCovarianceT = class {
  constructor(_Metadata = null, twist = null, covariance = []) {
    this._Metadata = _Metadata;
    this.twist = twist;
    this.covariance = covariance;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const twist = this.twist !== null ? this.twist.pack(builder) : 0;
    const covariance = TwistWithCovariance.createCovarianceVector(builder, this.covariance);
    TwistWithCovariance.startTwistWithCovariance(builder);
    TwistWithCovariance.add_Metadata(builder, _Metadata);
    TwistWithCovariance.addTwist(builder, twist);
    TwistWithCovariance.addCovariance(builder, covariance);
    return TwistWithCovariance.endTwistWithCovariance(builder);
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
  unpack() {
    return new Vector3StampedT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.vector() !== null ? this.vector().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.vector = this.vector() !== null ? this.vector().unpack() : null;
  }
};
var Vector3StampedT = class {
  constructor(_Metadata = null, header = null, vector = null) {
    this._Metadata = _Metadata;
    this.header = header;
    this.vector = vector;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const vector = this.vector !== null ? this.vector.pack(builder) : 0;
    Vector3Stamped.startVector3Stamped(builder);
    Vector3Stamped.add_Metadata(builder, _Metadata);
    Vector3Stamped.addHeader(builder, header);
    Vector3Stamped.addVector(builder, vector);
    return Vector3Stamped.endVector3Stamped(builder);
  }
};

// fbmsglib/src/fb/mavros-msgs.ts
var mavros_msgs_exports = {};
__export(mavros_msgs_exports, {
  Altitude: () => Altitude,
  AltitudeT: () => AltitudeT
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
  unpack() {
    return new AltitudeT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.monotonic(),
      this.amsl(),
      this.local(),
      this.relative(),
      this.terrain(),
      this.bottomClearance()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.monotonic = this.monotonic();
    _o.amsl = this.amsl();
    _o.local = this.local();
    _o.relative = this.relative();
    _o.terrain = this.terrain();
    _o.bottomClearance = this.bottomClearance();
  }
};
var AltitudeT = class {
  constructor(_Metadata = null, monotonic = 0, amsl = 0, local = 0, relative = 0, terrain = 0, bottomClearance = 0) {
    this._Metadata = _Metadata;
    this.monotonic = monotonic;
    this.amsl = amsl;
    this.local = local;
    this.relative = relative;
    this.terrain = terrain;
    this.bottomClearance = bottomClearance;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return Altitude.createAltitude(
      builder,
      _Metadata,
      this.monotonic,
      this.amsl,
      this.local,
      this.relative,
      this.terrain,
      this.bottomClearance
    );
  }
};

// fbmsglib/src/fb/nav-msgs.ts
var nav_msgs_exports = {};
__export(nav_msgs_exports, {
  Odometry: () => Odometry,
  OdometryT: () => OdometryT
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
  unpack() {
    return new OdometryT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.childFrameId(),
      this.pose() !== null ? this.pose().unpack() : null,
      this.twist() !== null ? this.twist().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.childFrameId = this.childFrameId();
    _o.pose = this.pose() !== null ? this.pose().unpack() : null;
    _o.twist = this.twist() !== null ? this.twist().unpack() : null;
  }
};
var OdometryT = class {
  constructor(_Metadata = null, header = null, childFrameId = null, pose = null, twist = null) {
    this._Metadata = _Metadata;
    this.header = header;
    this.childFrameId = childFrameId;
    this.pose = pose;
    this.twist = twist;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const childFrameId = this.childFrameId !== null ? builder.createString(this.childFrameId) : 0;
    const pose = this.pose !== null ? this.pose.pack(builder) : 0;
    const twist = this.twist !== null ? this.twist.pack(builder) : 0;
    Odometry.startOdometry(builder);
    Odometry.add_Metadata(builder, _Metadata);
    Odometry.addHeader(builder, header);
    Odometry.addChildFrameId(builder, childFrameId);
    Odometry.addPose(builder, pose);
    Odometry.addTwist(builder, twist);
    return Odometry.endOdometry(builder);
  }
};

// fbmsglib/src/fb/sensor-msgs.ts
var sensor_msgs_exports = {};
__export(sensor_msgs_exports, {
  BatteryState: () => BatteryState,
  BatteryStateT: () => BatteryStateT,
  CompressedImage: () => CompressedImage,
  CompressedImageT: () => CompressedImageT,
  Imu: () => Imu,
  ImuT: () => ImuT,
  LaserScan: () => LaserScan,
  LaserScanT: () => LaserScanT,
  NavSatFix: () => NavSatFix,
  NavSatFixT: () => NavSatFixT,
  NavSatStatus: () => NavSatStatus,
  NavSatStatusT: () => NavSatStatusT,
  PointCloud2: () => PointCloud2,
  PointCloud2T: () => PointCloud2T,
  PointField: () => PointField,
  PointFieldT: () => PointFieldT
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
  unpack() {
    return new BatteryStateT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.voltage(),
      this.current(),
      this.charge(),
      this.capacity(),
      this.designCapacity(),
      this.percentage(),
      this.powerSupplyStatus(),
      this.powerSupplyHealth(),
      this.powerSupplyTechnology(),
      this.present(),
      this.bb.createScalarList(this.cellVoltage.bind(this), this.cellVoltageLength()),
      this.bb.createScalarList(this.cellTemperature.bind(this), this.cellTemperatureLength())
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.voltage = this.voltage();
    _o.current = this.current();
    _o.charge = this.charge();
    _o.capacity = this.capacity();
    _o.designCapacity = this.designCapacity();
    _o.percentage = this.percentage();
    _o.powerSupplyStatus = this.powerSupplyStatus();
    _o.powerSupplyHealth = this.powerSupplyHealth();
    _o.powerSupplyTechnology = this.powerSupplyTechnology();
    _o.present = this.present();
    _o.cellVoltage = this.bb.createScalarList(this.cellVoltage.bind(this), this.cellVoltageLength());
    _o.cellTemperature = this.bb.createScalarList(this.cellTemperature.bind(this), this.cellTemperatureLength());
  }
};
var BatteryStateT = class {
  constructor(_Metadata = null, header = null, voltage = 0, current = 0, charge = 0, capacity = 0, designCapacity = 0, percentage = 0, powerSupplyStatus = 0, powerSupplyHealth = 0, powerSupplyTechnology = 0, present = false, cellVoltage = [], cellTemperature = []) {
    this._Metadata = _Metadata;
    this.header = header;
    this.voltage = voltage;
    this.current = current;
    this.charge = charge;
    this.capacity = capacity;
    this.designCapacity = designCapacity;
    this.percentage = percentage;
    this.powerSupplyStatus = powerSupplyStatus;
    this.powerSupplyHealth = powerSupplyHealth;
    this.powerSupplyTechnology = powerSupplyTechnology;
    this.present = present;
    this.cellVoltage = cellVoltage;
    this.cellTemperature = cellTemperature;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const cellVoltage = BatteryState.createCellVoltageVector(builder, this.cellVoltage);
    const cellTemperature = BatteryState.createCellTemperatureVector(builder, this.cellTemperature);
    BatteryState.startBatteryState(builder);
    BatteryState.add_Metadata(builder, _Metadata);
    BatteryState.addHeader(builder, header);
    BatteryState.addVoltage(builder, this.voltage);
    BatteryState.addCurrent(builder, this.current);
    BatteryState.addCharge(builder, this.charge);
    BatteryState.addCapacity(builder, this.capacity);
    BatteryState.addDesignCapacity(builder, this.designCapacity);
    BatteryState.addPercentage(builder, this.percentage);
    BatteryState.addPowerSupplyStatus(builder, this.powerSupplyStatus);
    BatteryState.addPowerSupplyHealth(builder, this.powerSupplyHealth);
    BatteryState.addPowerSupplyTechnology(builder, this.powerSupplyTechnology);
    BatteryState.addPresent(builder, this.present);
    BatteryState.addCellVoltage(builder, cellVoltage);
    BatteryState.addCellTemperature(builder, cellTemperature);
    return BatteryState.endBatteryState(builder);
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
  unpack() {
    return new CompressedImageT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.format(),
      this.bb.createScalarList(this.data.bind(this), this.dataLength())
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.format = this.format();
    _o.data = this.bb.createScalarList(this.data.bind(this), this.dataLength());
  }
};
var CompressedImageT = class {
  constructor(_Metadata = null, header = null, format = null, data = []) {
    this._Metadata = _Metadata;
    this.header = header;
    this.format = format;
    this.data = data;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const format = this.format !== null ? builder.createString(this.format) : 0;
    const data = CompressedImage.createDataVector(builder, this.data);
    CompressedImage.startCompressedImage(builder);
    CompressedImage.add_Metadata(builder, _Metadata);
    CompressedImage.addHeader(builder, header);
    CompressedImage.addFormat(builder, format);
    CompressedImage.addData(builder, data);
    return CompressedImage.endCompressedImage(builder);
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
  unpack() {
    return new ImuT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.orientation() !== null ? this.orientation().unpack() : null,
      this.bb.createScalarList(this.orientationCovariance.bind(this), this.orientationCovarianceLength()),
      this.angularVelocity() !== null ? this.angularVelocity().unpack() : null,
      this.bb.createScalarList(this.angularVelocityCovariance.bind(this), this.angularVelocityCovarianceLength()),
      this.linearAcceleration() !== null ? this.linearAcceleration().unpack() : null,
      this.bb.createScalarList(this.linearAccelerationCovariance.bind(this), this.linearAccelerationCovarianceLength())
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.orientation = this.orientation() !== null ? this.orientation().unpack() : null;
    _o.orientationCovariance = this.bb.createScalarList(this.orientationCovariance.bind(this), this.orientationCovarianceLength());
    _o.angularVelocity = this.angularVelocity() !== null ? this.angularVelocity().unpack() : null;
    _o.angularVelocityCovariance = this.bb.createScalarList(this.angularVelocityCovariance.bind(this), this.angularVelocityCovarianceLength());
    _o.linearAcceleration = this.linearAcceleration() !== null ? this.linearAcceleration().unpack() : null;
    _o.linearAccelerationCovariance = this.bb.createScalarList(this.linearAccelerationCovariance.bind(this), this.linearAccelerationCovarianceLength());
  }
};
var ImuT = class {
  constructor(_Metadata = null, header = null, orientation = null, orientationCovariance = [], angularVelocity = null, angularVelocityCovariance = [], linearAcceleration = null, linearAccelerationCovariance = []) {
    this._Metadata = _Metadata;
    this.header = header;
    this.orientation = orientation;
    this.orientationCovariance = orientationCovariance;
    this.angularVelocity = angularVelocity;
    this.angularVelocityCovariance = angularVelocityCovariance;
    this.linearAcceleration = linearAcceleration;
    this.linearAccelerationCovariance = linearAccelerationCovariance;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const orientation = this.orientation !== null ? this.orientation.pack(builder) : 0;
    const orientationCovariance = Imu.createOrientationCovarianceVector(builder, this.orientationCovariance);
    const angularVelocity = this.angularVelocity !== null ? this.angularVelocity.pack(builder) : 0;
    const angularVelocityCovariance = Imu.createAngularVelocityCovarianceVector(builder, this.angularVelocityCovariance);
    const linearAcceleration = this.linearAcceleration !== null ? this.linearAcceleration.pack(builder) : 0;
    const linearAccelerationCovariance = Imu.createLinearAccelerationCovarianceVector(builder, this.linearAccelerationCovariance);
    Imu.startImu(builder);
    Imu.add_Metadata(builder, _Metadata);
    Imu.addHeader(builder, header);
    Imu.addOrientation(builder, orientation);
    Imu.addOrientationCovariance(builder, orientationCovariance);
    Imu.addAngularVelocity(builder, angularVelocity);
    Imu.addAngularVelocityCovariance(builder, angularVelocityCovariance);
    Imu.addLinearAcceleration(builder, linearAcceleration);
    Imu.addLinearAccelerationCovariance(builder, linearAccelerationCovariance);
    return Imu.endImu(builder);
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
  unpack() {
    return new LaserScanT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.angleMin(),
      this.angleMax(),
      this.angleIncrement(),
      this.timeIncrement(),
      this.scanTime(),
      this.rangeMin(),
      this.rangeMax(),
      this.bb.createScalarList(this.ranges.bind(this), this.rangesLength()),
      this.bb.createScalarList(this.intensities.bind(this), this.intensitiesLength())
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.angleMin = this.angleMin();
    _o.angleMax = this.angleMax();
    _o.angleIncrement = this.angleIncrement();
    _o.timeIncrement = this.timeIncrement();
    _o.scanTime = this.scanTime();
    _o.rangeMin = this.rangeMin();
    _o.rangeMax = this.rangeMax();
    _o.ranges = this.bb.createScalarList(this.ranges.bind(this), this.rangesLength());
    _o.intensities = this.bb.createScalarList(this.intensities.bind(this), this.intensitiesLength());
  }
};
var LaserScanT = class {
  constructor(_Metadata = null, header = null, angleMin = 0, angleMax = 0, angleIncrement = 0, timeIncrement = 0, scanTime = 0, rangeMin = 0, rangeMax = 0, ranges = [], intensities = []) {
    this._Metadata = _Metadata;
    this.header = header;
    this.angleMin = angleMin;
    this.angleMax = angleMax;
    this.angleIncrement = angleIncrement;
    this.timeIncrement = timeIncrement;
    this.scanTime = scanTime;
    this.rangeMin = rangeMin;
    this.rangeMax = rangeMax;
    this.ranges = ranges;
    this.intensities = intensities;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const ranges = LaserScan.createRangesVector(builder, this.ranges);
    const intensities = LaserScan.createIntensitiesVector(builder, this.intensities);
    LaserScan.startLaserScan(builder);
    LaserScan.add_Metadata(builder, _Metadata);
    LaserScan.addHeader(builder, header);
    LaserScan.addAngleMin(builder, this.angleMin);
    LaserScan.addAngleMax(builder, this.angleMax);
    LaserScan.addAngleIncrement(builder, this.angleIncrement);
    LaserScan.addTimeIncrement(builder, this.timeIncrement);
    LaserScan.addScanTime(builder, this.scanTime);
    LaserScan.addRangeMin(builder, this.rangeMin);
    LaserScan.addRangeMax(builder, this.rangeMax);
    LaserScan.addRanges(builder, ranges);
    LaserScan.addIntensities(builder, intensities);
    return LaserScan.endLaserScan(builder);
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
  unpack() {
    return new PointFieldT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.name(),
      this.offset(),
      this.datatype(),
      this.count()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.name = this.name();
    _o.offset = this.offset();
    _o.datatype = this.datatype();
    _o.count = this.count();
  }
};
var PointFieldT = class {
  constructor(_Metadata = null, name = null, offset = 0, datatype = 0, count = 0) {
    this._Metadata = _Metadata;
    this.name = name;
    this.offset = offset;
    this.datatype = datatype;
    this.count = count;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const name = this.name !== null ? builder.createString(this.name) : 0;
    return PointField.createPointField(
      builder,
      _Metadata,
      name,
      this.offset,
      this.datatype,
      this.count
    );
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
  unpack() {
    return new PointCloud2T(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.header() !== null ? this.header().unpack() : null,
      this.height(),
      this.width(),
      this.bb.createObjList(this.fields.bind(this), this.fieldsLength()),
      this.isBigendian(),
      this.pointStep(),
      this.rowStep(),
      this.bb.createScalarList(this.data.bind(this), this.dataLength()),
      this.isDense()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.header = this.header() !== null ? this.header().unpack() : null;
    _o.height = this.height();
    _o.width = this.width();
    _o.fields = this.bb.createObjList(this.fields.bind(this), this.fieldsLength());
    _o.isBigendian = this.isBigendian();
    _o.pointStep = this.pointStep();
    _o.rowStep = this.rowStep();
    _o.data = this.bb.createScalarList(this.data.bind(this), this.dataLength());
    _o.isDense = this.isDense();
  }
};
var PointCloud2T = class {
  constructor(_Metadata = null, header = null, height = 0, width = 0, fields = [], isBigendian = false, pointStep = 0, rowStep = 0, data = [], isDense = false) {
    this._Metadata = _Metadata;
    this.header = header;
    this.height = height;
    this.width = width;
    this.fields = fields;
    this.isBigendian = isBigendian;
    this.pointStep = pointStep;
    this.rowStep = rowStep;
    this.data = data;
    this.isDense = isDense;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const header = this.header !== null ? this.header.pack(builder) : 0;
    const fields = PointCloud2.createFieldsVector(builder, builder.createObjectOffsetList(this.fields));
    const data = PointCloud2.createDataVector(builder, this.data);
    PointCloud2.startPointCloud2(builder);
    PointCloud2.add_Metadata(builder, _Metadata);
    PointCloud2.addHeader(builder, header);
    PointCloud2.addHeight(builder, this.height);
    PointCloud2.addWidth(builder, this.width);
    PointCloud2.addFields(builder, fields);
    PointCloud2.addIsBigendian(builder, this.isBigendian);
    PointCloud2.addPointStep(builder, this.pointStep);
    PointCloud2.addRowStep(builder, this.rowStep);
    PointCloud2.addData(builder, data);
    PointCloud2.addIsDense(builder, this.isDense);
    return PointCloud2.endPointCloud2(builder);
  }
};

// fbmsglib/src/fb/std-msgs.ts
var std_msgs_exports = {};
__export(std_msgs_exports, {
  Float32: () => Float32,
  Float32T: () => Float32T,
  Float64: () => Float64,
  Float64T: () => Float64T,
  Header: () => Header,
  HeaderT: () => HeaderT,
  String: () => String,
  StringT: () => StringT,
  UInt8: () => UInt8,
  UInt8T: () => UInt8T
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
  unpack() {
    return new Float32T(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.data()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.data = this.data();
  }
};
var Float32T = class {
  constructor(_Metadata = null, data = 0) {
    this._Metadata = _Metadata;
    this.data = data;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return Float32.createFloat32(
      builder,
      _Metadata,
      this.data
    );
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
  unpack() {
    return new Float64T(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.data()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.data = this.data();
  }
};
var Float64T = class {
  constructor(_Metadata = null, data = 0) {
    this._Metadata = _Metadata;
    this.data = data;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return Float64.createFloat64(
      builder,
      _Metadata,
      this.data
    );
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
  unpack() {
    return new StringT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.data()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.data = this.data();
  }
};
var StringT = class {
  constructor(_Metadata = null, data = null) {
    this._Metadata = _Metadata;
    this.data = data;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const data = this.data !== null ? builder.createString(this.data) : 0;
    return String.createString(
      builder,
      _Metadata,
      data
    );
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
  unpack() {
    return new UInt8T(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.data()
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.data = this.data();
  }
};
var UInt8T = class {
  constructor(_Metadata = null, data = 0) {
    this._Metadata = _Metadata;
    this.data = data;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    return UInt8.createUInt8(
      builder,
      _Metadata,
      this.data
    );
  }
};

// fbmsglib/src/fb/std-srvs.ts
var std_srvs_exports = {};
__export(std_srvs_exports, {
  Empty: () => Empty,
  EmptyRequest: () => EmptyRequest,
  EmptyRequestT: () => EmptyRequestT,
  EmptyResponse: () => EmptyResponse,
  EmptyResponseT: () => EmptyResponseT,
  EmptyT: () => EmptyT,
  SetBool: () => SetBool,
  SetBoolRequest: () => SetBoolRequest,
  SetBoolRequestT: () => SetBoolRequestT,
  SetBoolResponse: () => SetBoolResponse,
  SetBoolResponseT: () => SetBoolResponseT,
  SetBoolT: () => SetBoolT,
  Trigger: () => Trigger,
  TriggerRequest: () => TriggerRequest,
  TriggerRequestT: () => TriggerRequestT,
  TriggerResponse: () => TriggerResponse,
  TriggerResponseT: () => TriggerResponseT,
  TriggerT: () => TriggerT
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
  unpack() {
    return new EmptyRequestT();
  }
  unpackTo(_o) {
  }
};
var EmptyRequestT = class {
  constructor() {
  }
  pack(builder) {
    return EmptyRequest.createEmptyRequest(builder);
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
  unpack() {
    return new EmptyResponseT();
  }
  unpackTo(_o) {
  }
};
var EmptyResponseT = class {
  constructor() {
  }
  pack(builder) {
    return EmptyResponse.createEmptyResponse(builder);
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
  unpack() {
    return new EmptyT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.request() !== null ? this.request().unpack() : null,
      this.response() !== null ? this.response().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.request = this.request() !== null ? this.request().unpack() : null;
    _o.response = this.response() !== null ? this.response().unpack() : null;
  }
};
var EmptyT = class {
  constructor(_Metadata = null, request = null, response = null) {
    this._Metadata = _Metadata;
    this.request = request;
    this.response = response;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const request = this.request !== null ? this.request.pack(builder) : 0;
    const response = this.response !== null ? this.response.pack(builder) : 0;
    Empty.startEmpty(builder);
    Empty.add_Metadata(builder, _Metadata);
    Empty.addRequest(builder, request);
    Empty.addResponse(builder, response);
    return Empty.endEmpty(builder);
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
  unpack() {
    return new SetBoolRequestT(
      this.data()
    );
  }
  unpackTo(_o) {
    _o.data = this.data();
  }
};
var SetBoolRequestT = class {
  constructor(data = false) {
    this.data = data;
  }
  pack(builder) {
    return SetBoolRequest.createSetBoolRequest(
      builder,
      this.data
    );
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
  unpack() {
    return new SetBoolResponseT(
      this.success(),
      this.message()
    );
  }
  unpackTo(_o) {
    _o.success = this.success();
    _o.message = this.message();
  }
};
var SetBoolResponseT = class {
  constructor(success = false, message = null) {
    this.success = success;
    this.message = message;
  }
  pack(builder) {
    const message = this.message !== null ? builder.createString(this.message) : 0;
    return SetBoolResponse.createSetBoolResponse(
      builder,
      this.success,
      message
    );
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
  unpack() {
    return new SetBoolT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.request() !== null ? this.request().unpack() : null,
      this.response() !== null ? this.response().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.request = this.request() !== null ? this.request().unpack() : null;
    _o.response = this.response() !== null ? this.response().unpack() : null;
  }
};
var SetBoolT = class {
  constructor(_Metadata = null, request = null, response = null) {
    this._Metadata = _Metadata;
    this.request = request;
    this.response = response;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const request = this.request !== null ? this.request.pack(builder) : 0;
    const response = this.response !== null ? this.response.pack(builder) : 0;
    SetBool.startSetBool(builder);
    SetBool.add_Metadata(builder, _Metadata);
    SetBool.addRequest(builder, request);
    SetBool.addResponse(builder, response);
    return SetBool.endSetBool(builder);
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
  unpack() {
    return new TriggerRequestT();
  }
  unpackTo(_o) {
  }
};
var TriggerRequestT = class {
  constructor() {
  }
  pack(builder) {
    return TriggerRequest.createTriggerRequest(builder);
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
  unpack() {
    return new TriggerResponseT(
      this.success(),
      this.message()
    );
  }
  unpackTo(_o) {
    _o.success = this.success();
    _o.message = this.message();
  }
};
var TriggerResponseT = class {
  constructor(success = false, message = null) {
    this.success = success;
    this.message = message;
  }
  pack(builder) {
    const message = this.message !== null ? builder.createString(this.message) : 0;
    return TriggerResponse.createTriggerResponse(
      builder,
      this.success,
      message
    );
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
  unpack() {
    return new TriggerT(
      this._Metadata() !== null ? this._Metadata().unpack() : null,
      this.request() !== null ? this.request().unpack() : null,
      this.response() !== null ? this.response().unpack() : null
    );
  }
  unpackTo(_o) {
    _o._Metadata = this._Metadata() !== null ? this._Metadata().unpack() : null;
    _o.request = this.request() !== null ? this.request().unpack() : null;
    _o.response = this.response() !== null ? this.response().unpack() : null;
  }
};
var TriggerT = class {
  constructor(_Metadata = null, request = null, response = null) {
    this._Metadata = _Metadata;
    this.request = request;
    this.response = response;
  }
  pack(builder) {
    const _Metadata = this._Metadata !== null ? this._Metadata.pack(builder) : 0;
    const request = this.request !== null ? this.request.pack(builder) : 0;
    const response = this.response !== null ? this.response.pack(builder) : 0;
    Trigger.startTrigger(builder);
    Trigger.add_Metadata(builder, _Metadata);
    Trigger.addRequest(builder, request);
    Trigger.addResponse(builder, response);
    return Trigger.endTrigger(builder);
  }
};
// Annotate the CommonJS export names for ESM import in node:
0 && (module.exports = {
  fb
});
