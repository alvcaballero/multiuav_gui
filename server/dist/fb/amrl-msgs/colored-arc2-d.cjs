'use strict';
// automatically generated by the FlatBuffers compiler, do not modify
Object.defineProperty(exports, '__esModule', { value: true });
exports.ColoredArc2D = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
var flatbuffers = require('flatbuffers');
var msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
var point2_d_js_1 = require('../../fb/amrl-msgs/point2-d.cjs');
var ColoredArc2D = /** @class */ (function () {
  function ColoredArc2D() {
    this.bb = null;
    this.bb_pos = 0;
  }
  ColoredArc2D.prototype.__init = function (i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  };
  ColoredArc2D.getRootAsColoredArc2D = function (bb, obj) {
    return (obj || new ColoredArc2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  ColoredArc2D.getSizePrefixedRootAsColoredArc2D = function (bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new ColoredArc2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  ColoredArc2D.prototype._Metadata = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  ColoredArc2D.prototype.center = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 6);
    return offset
      ? (obj || new point2_d_js_1.Point2D()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  ColoredArc2D.prototype.radius = function () {
    var offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  };
  ColoredArc2D.prototype.startAngle = function () {
    var offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  };
  ColoredArc2D.prototype.endAngle = function () {
    var offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  };
  ColoredArc2D.prototype.color = function () {
    var offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  };
  ColoredArc2D.startColoredArc2D = function (builder) {
    builder.startObject(6);
  };
  ColoredArc2D.add_Metadata = function (builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  };
  ColoredArc2D.addCenter = function (builder, centerOffset) {
    builder.addFieldOffset(1, centerOffset, 0);
  };
  ColoredArc2D.addRadius = function (builder, radius) {
    builder.addFieldFloat32(2, radius, 0.0);
  };
  ColoredArc2D.addStartAngle = function (builder, startAngle) {
    builder.addFieldFloat32(3, startAngle, 0.0);
  };
  ColoredArc2D.addEndAngle = function (builder, endAngle) {
    builder.addFieldFloat32(4, endAngle, 0.0);
  };
  ColoredArc2D.addColor = function (builder, color) {
    builder.addFieldInt32(5, color, 0);
  };
  ColoredArc2D.endColoredArc2D = function (builder) {
    var offset = builder.endObject();
    builder.requiredField(offset, 6); // center
    return offset;
  };
  return ColoredArc2D;
})();
exports.ColoredArc2D = ColoredArc2D;
