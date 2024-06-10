'use strict';
// automatically generated by the FlatBuffers compiler, do not modify
Object.defineProperty(exports, '__esModule', { value: true });
exports.PointCloud2 = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
var flatbuffers = require('flatbuffers');
var msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
var point_field_js_1 = require('../../fb/sensor-msgs/point-field.cjs');
var header_js_1 = require('../../fb/std-msgs/header.cjs');
var PointCloud2 = /** @class */ (function () {
  function PointCloud2() {
    this.bb = null;
    this.bb_pos = 0;
  }
  PointCloud2.prototype.__init = function (i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  };
  PointCloud2.getRootAsPointCloud2 = function (bb, obj) {
    return (obj || new PointCloud2()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  PointCloud2.getSizePrefixedRootAsPointCloud2 = function (bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new PointCloud2()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  PointCloud2.prototype._Metadata = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  PointCloud2.prototype.header = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new header_js_1.Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  };
  PointCloud2.prototype.height = function () {
    var offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  };
  PointCloud2.prototype.width = function () {
    var offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  };
  PointCloud2.prototype.fields = function (index, obj) {
    var offset = this.bb.__offset(this.bb_pos, 12);
    return offset
      ? (obj || new point_field_js_1.PointField()).__init(
          this.bb.__indirect(this.bb.__vector(this.bb_pos + offset) + index * 4),
          this.bb
        )
      : null;
  };
  PointCloud2.prototype.fieldsLength = function () {
    var offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  };
  PointCloud2.prototype.isBigendian = function () {
    var offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  };
  PointCloud2.prototype.pointStep = function () {
    var offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  };
  PointCloud2.prototype.rowStep = function () {
    var offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  };
  PointCloud2.prototype.data = function (index) {
    var offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.readUint8(this.bb.__vector(this.bb_pos + offset) + index) : 0;
  };
  PointCloud2.prototype.dataLength = function () {
    var offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  };
  PointCloud2.prototype.dataArray = function () {
    var offset = this.bb.__offset(this.bb_pos, 20);
    return offset
      ? new Uint8Array(
          this.bb.bytes().buffer,
          this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset),
          this.bb.__vector_len(this.bb_pos + offset)
        )
      : null;
  };
  PointCloud2.prototype.isDense = function () {
    var offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  };
  PointCloud2.startPointCloud2 = function (builder) {
    builder.startObject(10);
  };
  PointCloud2.add_Metadata = function (builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  };
  PointCloud2.addHeader = function (builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  };
  PointCloud2.addHeight = function (builder, height) {
    builder.addFieldInt32(2, height, 0);
  };
  PointCloud2.addWidth = function (builder, width) {
    builder.addFieldInt32(3, width, 0);
  };
  PointCloud2.addFields = function (builder, fieldsOffset) {
    builder.addFieldOffset(4, fieldsOffset, 0);
  };
  PointCloud2.createFieldsVector = function (builder, data) {
    builder.startVector(4, data.length, 4);
    for (var i = data.length - 1; i >= 0; i--) {
      builder.addOffset(data[i]);
    }
    return builder.endVector();
  };
  PointCloud2.startFieldsVector = function (builder, numElems) {
    builder.startVector(4, numElems, 4);
  };
  PointCloud2.addIsBigendian = function (builder, isBigendian) {
    builder.addFieldInt8(5, +isBigendian, +false);
  };
  PointCloud2.addPointStep = function (builder, pointStep) {
    builder.addFieldInt32(6, pointStep, 0);
  };
  PointCloud2.addRowStep = function (builder, rowStep) {
    builder.addFieldInt32(7, rowStep, 0);
  };
  PointCloud2.addData = function (builder, dataOffset) {
    builder.addFieldOffset(8, dataOffset, 0);
  };
  PointCloud2.createDataVector = function (builder, data) {
    builder.startVector(1, data.length, 1);
    for (var i = data.length - 1; i >= 0; i--) {
      builder.addInt8(data[i]);
    }
    return builder.endVector();
  };
  PointCloud2.startDataVector = function (builder, numElems) {
    builder.startVector(1, numElems, 1);
  };
  PointCloud2.addIsDense = function (builder, isDense) {
    builder.addFieldInt8(9, +isDense, +false);
  };
  PointCloud2.endPointCloud2 = function (builder) {
    var offset = builder.endObject();
    builder.requiredField(offset, 6); // header
    builder.requiredField(offset, 12); // fields
    builder.requiredField(offset, 20); // data
    return offset;
  };
  return PointCloud2;
})();
exports.PointCloud2 = PointCloud2;