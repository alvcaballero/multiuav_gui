'use strict';
// automatically generated by the FlatBuffers compiler, do not modify
Object.defineProperty(exports, '__esModule', { value: true });
exports.NavSatStatus = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
var flatbuffers = require('flatbuffers');
var msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
var NavSatStatus = /** @class */ (function () {
  function NavSatStatus() {
    this.bb = null;
    this.bb_pos = 0;
  }
  NavSatStatus.prototype.__init = function (i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  };
  NavSatStatus.getRootAsNavSatStatus = function (bb, obj) {
    return (obj || new NavSatStatus()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  NavSatStatus.getSizePrefixedRootAsNavSatStatus = function (bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new NavSatStatus()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  NavSatStatus.prototype._Metadata = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  NavSatStatus.prototype.status = function () {
    var offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readInt8(this.bb_pos + offset) : 0;
  };
  NavSatStatus.prototype.service = function () {
    var offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint16(this.bb_pos + offset) : 0;
  };
  NavSatStatus.startNavSatStatus = function (builder) {
    builder.startObject(3);
  };
  NavSatStatus.add_Metadata = function (builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  };
  NavSatStatus.addStatus = function (builder, status) {
    builder.addFieldInt8(1, status, 0);
  };
  NavSatStatus.addService = function (builder, service) {
    builder.addFieldInt16(2, service, 0);
  };
  NavSatStatus.endNavSatStatus = function (builder) {
    var offset = builder.endObject();
    return offset;
  };
  NavSatStatus.createNavSatStatus = function (builder, _MetadataOffset, status, service) {
    NavSatStatus.startNavSatStatus(builder);
    NavSatStatus.add_Metadata(builder, _MetadataOffset);
    NavSatStatus.addStatus(builder, status);
    NavSatStatus.addService(builder, service);
    return NavSatStatus.endNavSatStatus(builder);
  };
  return NavSatStatus;
})();
exports.NavSatStatus = NavSatStatus;