'use strict';
// automatically generated by the FlatBuffers compiler, do not modify
Object.defineProperty(exports, '__esModule', { value: true });
exports.RobofleetStatus = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
var flatbuffers = require('flatbuffers');
var msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
var RobofleetStatus = /** @class */ (function () {
  function RobofleetStatus() {
    this.bb = null;
    this.bb_pos = 0;
  }
  RobofleetStatus.prototype.__init = function (i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  };
  RobofleetStatus.getRootAsRobofleetStatus = function (bb, obj) {
    return (obj || new RobofleetStatus()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  RobofleetStatus.getSizePrefixedRootAsRobofleetStatus = function (bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new RobofleetStatus()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  RobofleetStatus.prototype._Metadata = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  RobofleetStatus.prototype.status = function (optionalEncoding) {
    var offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  };
  RobofleetStatus.prototype.isOk = function () {
    var offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  };
  RobofleetStatus.prototype.batteryLevel = function () {
    var offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  };
  RobofleetStatus.prototype.location = function (optionalEncoding) {
    var offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  };
  RobofleetStatus.startRobofleetStatus = function (builder) {
    builder.startObject(5);
  };
  RobofleetStatus.add_Metadata = function (builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  };
  RobofleetStatus.addStatus = function (builder, statusOffset) {
    builder.addFieldOffset(1, statusOffset, 0);
  };
  RobofleetStatus.addIsOk = function (builder, isOk) {
    builder.addFieldInt8(2, +isOk, +false);
  };
  RobofleetStatus.addBatteryLevel = function (builder, batteryLevel) {
    builder.addFieldFloat32(3, batteryLevel, 0.0);
  };
  RobofleetStatus.addLocation = function (builder, locationOffset) {
    builder.addFieldOffset(4, locationOffset, 0);
  };
  RobofleetStatus.endRobofleetStatus = function (builder) {
    var offset = builder.endObject();
    builder.requiredField(offset, 6); // status
    builder.requiredField(offset, 12); // location
    return offset;
  };
  RobofleetStatus.createRobofleetStatus = function (
    builder,
    _MetadataOffset,
    statusOffset,
    isOk,
    batteryLevel,
    locationOffset
  ) {
    RobofleetStatus.startRobofleetStatus(builder);
    RobofleetStatus.add_Metadata(builder, _MetadataOffset);
    RobofleetStatus.addStatus(builder, statusOffset);
    RobofleetStatus.addIsOk(builder, isOk);
    RobofleetStatus.addBatteryLevel(builder, batteryLevel);
    RobofleetStatus.addLocation(builder, locationOffset);
    return RobofleetStatus.endRobofleetStatus(builder);
  };
  return RobofleetStatus;
})();
exports.RobofleetStatus = RobofleetStatus;
