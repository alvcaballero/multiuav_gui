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
exports.ColoredLine2D = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
const flatbuffers = __importStar(require('flatbuffers'));
const msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
const point2_d_js_1 = require('../../fb/amrl-msgs/point2-d.cjs');
class ColoredLine2D {
  constructor() {
    this.bb = null;
    this.bb_pos = 0;
  }
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsColoredLine2D(bb, obj) {
    return (obj || new ColoredLine2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsColoredLine2D(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new ColoredLine2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  }
  p0(obj) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset
      ? (obj || new point2_d_js_1.Point2D()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  }
  p1(obj) {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset
      ? (obj || new point2_d_js_1.Point2D()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  }
  color() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readUint32(this.bb_pos + offset) : 0;
  }
  static startColoredLine2D(builder) {
    builder.startObject(4);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addP0(builder, p0Offset) {
    builder.addFieldOffset(1, p0Offset, 0);
  }
  static addP1(builder, p1Offset) {
    builder.addFieldOffset(2, p1Offset, 0);
  }
  static addColor(builder, color) {
    builder.addFieldInt32(3, color, 0);
  }
  static endColoredLine2D(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6); // p0
    builder.requiredField(offset, 8); // p1
    return offset;
  }
}
exports.ColoredLine2D = ColoredLine2D;
