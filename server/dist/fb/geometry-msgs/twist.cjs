'use strict';
// automatically generated by the FlatBuffers compiler, do not modify
Object.defineProperty(exports, '__esModule', { value: true });
exports.Twist = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
var flatbuffers = require('flatbuffers');
var msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
var vector3_js_1 = require('../../fb/geometry-msgs/vector3.cjs');
var Twist = /** @class */ (function () {
  function Twist() {
    this.bb = null;
    this.bb_pos = 0;
  }
  Twist.prototype.__init = function (i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  };
  Twist.getRootAsTwist = function (bb, obj) {
    return (obj || new Twist()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  Twist.getSizePrefixedRootAsTwist = function (bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new Twist()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  Twist.prototype._Metadata = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  Twist.prototype.linear = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 6);
    return offset
      ? (obj || new vector3_js_1.Vector3()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  Twist.prototype.angular = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 8);
    return offset
      ? (obj || new vector3_js_1.Vector3()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  Twist.startTwist = function (builder) {
    builder.startObject(3);
  };
  Twist.add_Metadata = function (builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  };
  Twist.addLinear = function (builder, linearOffset) {
    builder.addFieldOffset(1, linearOffset, 0);
  };
  Twist.addAngular = function (builder, angularOffset) {
    builder.addFieldOffset(2, angularOffset, 0);
  };
  Twist.endTwist = function (builder) {
    var offset = builder.endObject();
    builder.requiredField(offset, 6); // linear
    builder.requiredField(offset, 8); // angular
    return offset;
  };
  return Twist;
})();
exports.Twist = Twist;
