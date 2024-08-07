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
exports.RobofleetSubscription = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
const flatbuffers = __importStar(require('flatbuffers'));
const msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
class RobofleetSubscription {
  constructor() {
    this.bb = null;
    this.bb_pos = 0;
  }
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsRobofleetSubscription(bb, obj) {
    return (obj || new RobofleetSubscription()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsRobofleetSubscription(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new RobofleetSubscription()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  _Metadata(obj) {
    const offset = this.bb.__offset(this.bb_pos, 4);
    return offset
      ? (obj || new msg_metadata_js_1.MsgMetadata()).__init(this.bb.__indirect(this.bb_pos + offset), this.bb)
      : null;
  }
  topicRegex(optionalEncoding) {
    const offset = this.bb.__offset(this.bb_pos, 6);
    return offset ? this.bb.__string(this.bb_pos + offset, optionalEncoding) : null;
  }
  action() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readUint8(this.bb_pos + offset) : 0;
  }
  static startRobofleetSubscription(builder) {
    builder.startObject(3);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addTopicRegex(builder, topicRegexOffset) {
    builder.addFieldOffset(1, topicRegexOffset, 0);
  }
  static addAction(builder, action) {
    builder.addFieldInt8(2, action, 0);
  }
  static endRobofleetSubscription(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6); // topic_regex
    return offset;
  }
  static createRobofleetSubscription(builder, _MetadataOffset, topicRegexOffset, action) {
    RobofleetSubscription.startRobofleetSubscription(builder);
    RobofleetSubscription.add_Metadata(builder, _MetadataOffset);
    RobofleetSubscription.addTopicRegex(builder, topicRegexOffset);
    RobofleetSubscription.addAction(builder, action);
    return RobofleetSubscription.endRobofleetSubscription(builder);
  }
}
exports.RobofleetSubscription = RobofleetSubscription;
