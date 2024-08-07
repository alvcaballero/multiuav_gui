'use strict';
// automatically generated by the FlatBuffers compiler, do not modify
var __createBinding =
  (this && this.__createBinding) ||
  (Object.create
    ? function (o, m, k, k2) {
        if (k2 === undefined) k2 = k;
        var desc = Object.getOwnPropertyDescriptor(m, k);
        if (!desc || ('get' in desc ? !m.__esModule : desc.writable || desc.configurable)) {
          desc = {
            enumerable: true,
            get: function () {
              return m[k];
            },
          };
        }
        Object.defineProperty(o, k2, desc);
      }
    : function (o, m, k, k2) {
        if (k2 === undefined) k2 = k;
        o[k2] = m[k];
      });
var __setModuleDefault =
  (this && this.__setModuleDefault) ||
  (Object.create
    ? function (o, v) {
        Object.defineProperty(o, 'default', { enumerable: true, value: v });
      }
    : function (o, v) {
        o['default'] = v;
      });
var __importStar =
  (this && this.__importStar) ||
  function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null)
      for (var k in mod)
        if (k !== 'default' && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
  };
Object.defineProperty(exports, '__esModule', { value: true });
exports.Odometry = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
const flatbuffers = __importStar(require('flatbuffers'));
const msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
const pose_with_covariance_js_1 = require('../../fb/geometry-msgs/pose-with-covariance.cjs');
const twist_with_covariance_js_1 = require('../../fb/geometry-msgs/twist-with-covariance.cjs');
const header_js_1 = require('../../fb/std-msgs/header.cjs');
class Odometry {
  constructor() {
    this.bb = null;
    this.bb_pos = 0;
  }
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsOdometry(bb, obj) {
    return (obj || new Odometry()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsOdometry(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new Odometry()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  }
  header(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new header_js_1.Header()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  childFrameId(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  pose(obj) {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset
      ? (obj || new pose_with_covariance_js_1.PoseWithCovariance()).__init(
          this.bb.__indirect(this.bb_pos + offset),
          this.bb
        )
      : null;
  }
  twist(obj) {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset
      ? (obj || new twist_with_covariance_js_1.TwistWithCovariance()).__init(
          this.bb.__indirect(this.bb_pos + offset),
          this.bb
        )
      : null;
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
    builder.requiredField(offset, 6); // header
    builder.requiredField(offset, 8); // child_frame_id
    builder.requiredField(offset, 10); // pose
    builder.requiredField(offset, 12); // twist
    return offset;
  }
}
exports.Odometry = Odometry;
