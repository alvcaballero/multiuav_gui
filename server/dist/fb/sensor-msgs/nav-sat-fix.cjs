'use strict';
// automatically generated by the FlatBuffers compiler, do not modify
Object.defineProperty(exports, '__esModule', { value: true });
exports.NavSatFix = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
var flatbuffers = require('flatbuffers');
var msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
var nav_sat_status_js_1 = require('../../fb/sensor-msgs/nav-sat-status.cjs');
var header_js_1 = require('../../fb/std-msgs/header.cjs');
var NavSatFix = /** @class */ (function () {
  function NavSatFix() {
    this.bb = null;
    this.bb_pos = 0;
  }
  NavSatFix.prototype.__init = function (i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  };
  NavSatFix.getRootAsNavSatFix = function (bb, obj) {
    return (obj || new NavSatFix()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  NavSatFix.getSizePrefixedRootAsNavSatFix = function (bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new NavSatFix()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  NavSatFix.prototype._Metadata = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  NavSatFix.prototype.header = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new header_js_1.Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  };
  NavSatFix.prototype.status = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 8);
    return offset
      ? (obj || new nav_sat_status_js_1.NavSatStatus()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  NavSatFix.prototype.latitude = function () {
    var offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0.0;
  };
  NavSatFix.prototype.longitude = function () {
    var offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0.0;
  };
  NavSatFix.prototype.altitude = function () {
    var offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat64(this.bb_pos + offset) : 0.0;
  };
  NavSatFix.prototype.positionCovariance = function (index) {
    var offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readFloat64(this.bb.__vector(this.bb_pos + offset) + index * 8) : 0;
  };
  NavSatFix.prototype.positionCovarianceLength = function () {
    var offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  };
  NavSatFix.prototype.positionCovarianceArray = function () {
    var offset = this.bb.__offset(this.bb_pos, 16);
    return offset
      ? new Float64Array(
          this.bb.bytes().buffer,
          this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset),
          this.bb.__vector_len(this.bb_pos + offset)
        )
      : null;
  };
  NavSatFix.prototype.positionCovarianceType = function () {
    var offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  };
  NavSatFix.startNavSatFix = function (builder) {
    builder.startObject(8);
  };
  NavSatFix.add_Metadata = function (builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  };
  NavSatFix.addHeader = function (builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  };
  NavSatFix.addStatus = function (builder, statusOffset) {
    builder.addFieldOffset(2, statusOffset, 0);
  };
  NavSatFix.addLatitude = function (builder, latitude) {
    builder.addFieldFloat64(3, latitude, 0.0);
  };
  NavSatFix.addLongitude = function (builder, longitude) {
    builder.addFieldFloat64(4, longitude, 0.0);
  };
  NavSatFix.addAltitude = function (builder, altitude) {
    builder.addFieldFloat64(5, altitude, 0.0);
  };
  NavSatFix.addPositionCovariance = function (builder, positionCovarianceOffset) {
    builder.addFieldOffset(6, positionCovarianceOffset, 0);
  };
  NavSatFix.createPositionCovarianceVector = function (builder, data) {
    builder.startVector(8, data.length, 8);
    for (var i = data.length - 1; i >= 0; i--) {
      builder.addFloat64(data[i]);
    }
    return builder.endVector();
  };
  NavSatFix.startPositionCovarianceVector = function (builder, numElems) {
    builder.startVector(8, numElems, 8);
  };
  NavSatFix.addPositionCovarianceType = function (builder, positionCovarianceType) {
    builder.addFieldInt8(7, positionCovarianceType, 0);
  };
  NavSatFix.endNavSatFix = function (builder) {
    var offset = builder.endObject();
    builder.requiredField(offset, 6); // header
    builder.requiredField(offset, 8); // status
    builder.requiredField(offset, 16); // position_covariance
    return offset;
  };
  return NavSatFix;
})();
exports.NavSatFix = NavSatFix;