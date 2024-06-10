// automatically generated by the FlatBuffers compiler, do not modify

/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */

import * as flatbuffers from 'flatbuffers';

import { MsgMetadata } from '../../fb/msg-metadata.js';


export class Vector3 {
  bb: flatbuffers.ByteBuffer|null = null;
  bb_pos = 0;
  __init(i:number, bb:flatbuffers.ByteBuffer):Vector3 {
  this.bb_pos = i;
  this.bb = bb;
  return this;
}

static getRootAsVector3(bb:flatbuffers.ByteBuffer, obj?:Vector3):Vector3 {
  return (obj || new Vector3()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

static getSizePrefixedRootAsVector3(bb:flatbuffers.ByteBuffer, obj?:Vector3):Vector3 {
  bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
  return (obj || new Vector3()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

_Metadata(obj?:MsgMetadata):MsgMetadata|null {
  const offset = this.bb!.__offset(this.bb_pos, 4);
  return offset ? (obj || new MsgMetadata()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

x():number {
  const offset = this.bb!.__offset(this.bb_pos, 6);
  return offset ? this.bb!.readFloat64(this.bb_pos + offset) : 0.0;
}

y():number {
  const offset = this.bb!.__offset(this.bb_pos, 8);
  return offset ? this.bb!.readFloat64(this.bb_pos + offset) : 0.0;
}

z():number {
  const offset = this.bb!.__offset(this.bb_pos, 10);
  return offset ? this.bb!.readFloat64(this.bb_pos + offset) : 0.0;
}

static startVector3(builder:flatbuffers.Builder) {
  builder.startObject(4);
}

static add_Metadata(builder:flatbuffers.Builder, _MetadataOffset:flatbuffers.Offset) {
  builder.addFieldOffset(0, _MetadataOffset, 0);
}

static addX(builder:flatbuffers.Builder, x:number) {
  builder.addFieldFloat64(1, x, 0.0);
}

static addY(builder:flatbuffers.Builder, y:number) {
  builder.addFieldFloat64(2, y, 0.0);
}

static addZ(builder:flatbuffers.Builder, z:number) {
  builder.addFieldFloat64(3, z, 0.0);
}

static endVector3(builder:flatbuffers.Builder):flatbuffers.Offset {
  const offset = builder.endObject();
  return offset;
}

static createVector3(builder:flatbuffers.Builder, _MetadataOffset:flatbuffers.Offset, x:number, y:number, z:number):flatbuffers.Offset {
  Vector3.startVector3(builder);
  Vector3.add_Metadata(builder, _MetadataOffset);
  Vector3.addX(builder, x);
  Vector3.addY(builder, y);
  Vector3.addZ(builder, z);
  return Vector3.endVector3(builder);
}
}