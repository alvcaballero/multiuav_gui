// automatically generated by the FlatBuffers compiler, do not modify

/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */

import * as flatbuffers from 'flatbuffers';

import { MsgMetadata } from '../../fb/msg-metadata.js';
import { Pose2Df } from '../../fb/amrl-msgs/pose2-df.js';
import { Header } from '../../fb/std-msgs/header.js';


export class Localization2DMsg {
  bb: flatbuffers.ByteBuffer|null = null;
  bb_pos = 0;
  __init(i:number, bb:flatbuffers.ByteBuffer):Localization2DMsg {
  this.bb_pos = i;
  this.bb = bb;
  return this;
}

static getRootAsLocalization2DMsg(bb:flatbuffers.ByteBuffer, obj?:Localization2DMsg):Localization2DMsg {
  return (obj || new Localization2DMsg()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

static getSizePrefixedRootAsLocalization2DMsg(bb:flatbuffers.ByteBuffer, obj?:Localization2DMsg):Localization2DMsg {
  bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
  return (obj || new Localization2DMsg()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

_Metadata(obj?:MsgMetadata):MsgMetadata|null {
  const offset = this.bb!.__offset(this.bb_pos, 4);
  return offset ? (obj || new MsgMetadata()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

header(obj?:Header):Header|null {
  const offset = this.bb!.__offset(this.bb_pos, 6);
  return offset ? (obj || new Header()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

pose(obj?:Pose2Df):Pose2Df|null {
  const offset = this.bb!.__offset(this.bb_pos, 8);
  return offset ? (obj || new Pose2Df()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

map():string|null
map(optionalEncoding:flatbuffers.Encoding):string|Uint8Array|null
map(optionalEncoding?:any):string|Uint8Array|null {
  const offset = this.bb!.__offset(this.bb_pos, 10);
  return offset ? this.bb!.__string(this.bb_pos + offset, optionalEncoding) : null;
}

static startLocalization2DMsg(builder:flatbuffers.Builder) {
  builder.startObject(4);
}

static add_Metadata(builder:flatbuffers.Builder, _MetadataOffset:flatbuffers.Offset) {
  builder.addFieldOffset(0, _MetadataOffset, 0);
}

static addHeader(builder:flatbuffers.Builder, headerOffset:flatbuffers.Offset) {
  builder.addFieldOffset(1, headerOffset, 0);
}

static addPose(builder:flatbuffers.Builder, poseOffset:flatbuffers.Offset) {
  builder.addFieldOffset(2, poseOffset, 0);
}

static addMap(builder:flatbuffers.Builder, mapOffset:flatbuffers.Offset) {
  builder.addFieldOffset(3, mapOffset, 0);
}

static endLocalization2DMsg(builder:flatbuffers.Builder):flatbuffers.Offset {
  const offset = builder.endObject();
  builder.requiredField(offset, 6) // header
  builder.requiredField(offset, 8) // pose
  builder.requiredField(offset, 10) // map
  return offset;
}

}
