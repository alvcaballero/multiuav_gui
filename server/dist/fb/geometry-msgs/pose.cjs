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
exports.Pose = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
const flatbuffers = __importStar(require('flatbuffers'));
const msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
const point_js_1 = require('../../fb/geometry-msgs/point.cjs');
const quaternion_js_1 = require('../../fb/geometry-msgs/quaternion.cjs');
class Pose {
  constructor() {
    this.bb = null;
    this.bb_pos = 0;
  }
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsPose(bb, obj) {
    return (obj || new Pose()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsPose(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new Pose()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  }
  position(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? (obj || new point_js_1.Point()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb) : null;
  }
  orientation(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset
      ? (obj || new quaternion_js_1.Quaternion()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  }
  static startPose(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addPosition(builder, positionOffset) {
    builder.addFieldOffset(1, positionOffset, 0);
  }
  static addOrientation(builder, orientationOffset) {
    builder.addFieldOffset(2, orientationOffset, 0);
  }
  static endPose(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6); // position
    builder.requiredField(offset, 8); // orientation
    return offset;
  }
}
exports.Pose = Pose;
