'use strict';
// automatically generated by the FlatBuffers compiler, do not modify
Object.defineProperty(exports, '__esModule', { value: true });
exports.Altitude = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
var flatbuffers = require('flatbuffers');
var msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
var Altitude = /** @class */ (function () {
  function Altitude() {
    this.bb = null;
    this.bb_pos = 0;
  }
  Altitude.prototype.__init = function (i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  };
  Altitude.getRootAsAltitude = function (bb, obj) {
    return (obj || new Altitude()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  Altitude.getSizePrefixedRootAsAltitude = function (bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new Altitude()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  Altitude.prototype._Metadata = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  Altitude.prototype.monotonic = function () {
    var offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  };
  Altitude.prototype.amsl = function () {
    var offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  };
  Altitude.prototype.local = function () {
    var offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  };
  Altitude.prototype.relative = function () {
    var offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  };
  Altitude.prototype.terrain = function () {
    var offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  };
  Altitude.prototype.bottomClearance = function () {
    var offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  };
  Altitude.startAltitude = function (builder) {
    builder.startObject(7);
  };
  Altitude.add_Metadata = function (builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  };
  Altitude.addMonotonic = function (builder, monotonic) {
    builder.addFieldFloat32(1, monotonic, 0.0);
  };
  Altitude.addAmsl = function (builder, amsl) {
    builder.addFieldFloat32(2, amsl, 0.0);
  };
  Altitude.addLocal = function (builder, local) {
    builder.addFieldFloat32(3, local, 0.0);
  };
  Altitude.addRelative = function (builder, relative) {
    builder.addFieldFloat32(4, relative, 0.0);
  };
  Altitude.addTerrain = function (builder, terrain) {
    builder.addFieldFloat32(5, terrain, 0.0);
  };
  Altitude.addBottomClearance = function (builder, bottomClearance) {
    builder.addFieldFloat32(6, bottomClearance, 0.0);
  };
  Altitude.endAltitude = function (builder) {
    var offset = builder.endObject();
    return offset;
  };
  Altitude.createAltitude = function (
    builder,
    _MetadataOffset,
    monotonic,
    amsl,
    local,
    relative,
    terrain,
    bottomClearance
  ) {
    Altitude.startAltitude(builder);
    Altitude.add_Metadata(builder, _MetadataOffset);
    Altitude.addMonotonic(builder, monotonic);
    Altitude.addAmsl(builder, amsl);
    Altitude.addLocal(builder, local);
    Altitude.addRelative(builder, relative);
    Altitude.addTerrain(builder, terrain);
    Altitude.addBottomClearance(builder, bottomClearance);
    return Altitude.endAltitude(builder);
  };
  return Altitude;
})();
exports.Altitude = Altitude;