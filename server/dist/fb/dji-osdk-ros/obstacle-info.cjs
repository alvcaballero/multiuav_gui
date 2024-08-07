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
exports.ObstacleInfo = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
const flatbuffers = __importStar(require('flatbuffers'));
const msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
const header_js_1 = require('../../fb/std-msgs/header.cjs');
class ObstacleInfo {
  constructor() {
    this.bb = null;
    this.bb_pos = 0;
  }
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsObstacleInfo(bb, obj) {
    return (obj || new ObstacleInfo()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsObstacleInfo(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new ObstacleInfo()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
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
  down() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  front() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  right() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  back() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  left() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  up() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  healtNotWorking() {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  healtWorking() {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 1;
  }
  downHealth() {
    const offset = this.bb.__offset(this.bb_pos, 24);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  frontHealth() {
    const offset = this.bb.__offset(this.bb_pos, 26);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  rightHealth() {
    const offset = this.bb.__offset(this.bb_pos, 28);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  backHealth() {
    const offset = this.bb.__offset(this.bb_pos, 30);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  leftHealth() {
    const offset = this.bb.__offset(this.bb_pos, 32);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  upHealth() {
    const offset = this.bb.__offset(this.bb_pos, 34);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  static startObstacleInfo(builder) {
    builder.startObject(16);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addDown(builder, down) {
    builder.addFieldFloat32(2, down, 0.0);
  }
  static addFront(builder, front) {
    builder.addFieldFloat32(3, front, 0.0);
  }
  static addRight(builder, right) {
    builder.addFieldFloat32(4, right, 0.0);
  }
  static addBack(builder, back) {
    builder.addFieldFloat32(5, back, 0.0);
  }
  static addLeft(builder, left) {
    builder.addFieldFloat32(6, left, 0.0);
  }
  static addUp(builder, up) {
    builder.addFieldFloat32(7, up, 0.0);
  }
  static addHealtNotWorking(builder, healtNotWorking) {
    builder.addFieldInt8(8, healtNotWorking, 0);
  }
  static addHealtWorking(builder, healtWorking) {
    builder.addFieldInt8(9, healtWorking, 1);
  }
  static addDownHealth(builder, downHealth) {
    builder.addFieldInt8(10, downHealth, 0);
  }
  static addFrontHealth(builder, frontHealth) {
    builder.addFieldInt8(11, frontHealth, 0);
  }
  static addRightHealth(builder, rightHealth) {
    builder.addFieldInt8(12, rightHealth, 0);
  }
  static addBackHealth(builder, backHealth) {
    builder.addFieldInt8(13, backHealth, 0);
  }
  static addLeftHealth(builder, leftHealth) {
    builder.addFieldInt8(14, leftHealth, 0);
  }
  static addUpHealth(builder, upHealth) {
    builder.addFieldInt8(15, upHealth, 0);
  }
  static endObstacleInfo(builder) {
    const offset = builder.endObject();
    return offset;
  }
}
exports.ObstacleInfo = ObstacleInfo;
