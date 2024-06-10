'use strict';
// automatically generated by the FlatBuffers compiler, do not modify
Object.defineProperty(exports, '__esModule', { value: true });
exports.Vector3Stamped = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
var flatbuffers = require('flatbuffers');
var msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
var vector3_js_1 = require('../../fb/geometry-msgs/vector3.cjs');
var header_js_1 = require('../../fb/std-msgs/header.cjs');
var Vector3Stamped = /** @class */ (function () {
  function Vector3Stamped() {
    this.bb = null;
    this.bb_pos = 0;
  }
  Vector3Stamped.prototype.__init = function (i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  };
  Vector3Stamped.getRootAsVector3Stamped = function (bb, obj) {
    return (obj || new Vector3Stamped()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  Vector3Stamped.getSizePrefixedRootAsVector3Stamped = function (bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new Vector3Stamped()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  Vector3Stamped.prototype._Metadata = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  Vector3Stamped.prototype.header = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new header_js_1.Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  };
  Vector3Stamped.prototype.vector = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 8);
    return offset
      ? (obj || new vector3_js_1.Vector3()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  Vector3Stamped.startVector3Stamped = function (builder) {
    builder.startObject(3);
  };
  Vector3Stamped.add_Metadata = function (builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  };
  Vector3Stamped.addHeader = function (builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  };
  Vector3Stamped.addVector = function (builder, vectorOffset) {
    builder.addFieldOffset(2, vectorOffset, 0);
  };
  Vector3Stamped.endVector3Stamped = function (builder) {
    var offset = builder.endObject();
    builder.requiredField(offset, 6); // header
    builder.requiredField(offset, 8); // vector
    return offset;
  };
  return Vector3Stamped;
})();
exports.Vector3Stamped = Vector3Stamped;