'use strict';
// automatically generated by the FlatBuffers compiler, do not modify
Object.defineProperty(exports, '__esModule', { value: true });
exports.Odometry = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
var flatbuffers = require('flatbuffers');
var msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
var pose_with_covariance_js_1 = require('../../fb/geometry-msgs/pose-with-covariance.cjs');
var twist_with_covariance_js_1 = require('../../fb/geometry-msgs/twist-with-covariance.cjs');
var header_js_1 = require('../../fb/std-msgs/header.cjs');
var Odometry = /** @class */ (function () {
  function Odometry() {
    this.bb = null;
    this.bb_pos = 0;
  }
  Odometry.prototype.__init = function (i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  };
  Odometry.getRootAsOdometry = function (bb, obj) {
    return (obj || new Odometry()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  Odometry.getSizePrefixedRootAsOdometry = function (bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new Odometry()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  };
  Odometry.prototype._Metadata = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  };
  Odometry.prototype.header = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new header_js_1.Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  };
  Odometry.prototype.childFrameId = function (optionalEncoding) {
    var offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  };
  Odometry.prototype.pose = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 10);
    return offset
      ? (obj || new pose_with_covariance_js_1.PoseWithCovariance()).__init(
          this.bb.__indirect(this.bb_pos + offset),
          this.bb
        )
      : null;
  };
  Odometry.prototype.twist = function (obj) {
    var offset = this.bb.__offset(this.bb_pos, 12);
    return offset
      ? (obj || new twist_with_covariance_js_1.TwistWithCovariance()).__init(
          this.bb.__indirect(this.bb_pos + offset),
          this.bb
        )
      : null;
  };
  Odometry.startOdometry = function (builder) {
    builder.startObject(5);
  };
  Odometry.add_Metadata = function (builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  };
  Odometry.addHeader = function (builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  };
  Odometry.addChildFrameId = function (builder, childFrameIdOffset) {
    builder.addFieldOffset(2, childFrameIdOffset, 0);
  };
  Odometry.addPose = function (builder, poseOffset) {
    builder.addFieldOffset(3, poseOffset, 0);
  };
  Odometry.addTwist = function (builder, twistOffset) {
    builder.addFieldOffset(4, twistOffset, 0);
  };
  Odometry.endOdometry = function (builder) {
    var offset = builder.endObject();
    builder.requiredField(offset, 6); // header
    builder.requiredField(offset, 8); // child_frame_id
    builder.requiredField(offset, 10); // pose
    builder.requiredField(offset, 12); // twist
    return offset;
  };
  return Odometry;
})();
exports.Odometry = Odometry;
