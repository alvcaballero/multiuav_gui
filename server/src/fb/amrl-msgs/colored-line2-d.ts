// automatically generated by the FlatBuffers compiler, do not modify

/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */

import * as flatbuffers from 'flatbuffers';

import { MsgMetadata } from '../../fb/msg-metadata.js';
import { Point2D } from '../../fb/amrl-msgs/point2-d.js';


export class ColoredLine2D {
  bb: flatbuffers.ByteBuffer|null = null;
  bb_pos = 0;
  __init(i:number, bb:flatbuffers.ByteBuffer):ColoredLine2D {
  this.bb_pos = i;
  this.bb = bb;
  return this;
}

static getRootAsColoredLine2D(bb:flatbuffers.ByteBuffer, obj?:ColoredLine2D):ColoredLine2D {
  return (obj || new ColoredLine2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

static getSizePrefixedRootAsColoredLine2D(bb:flatbuffers.ByteBuffer, obj?:ColoredLine2D):ColoredLine2D {
  bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
  return (obj || new ColoredLine2D()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

_Metadata(obj?:MsgMetadata):MsgMetadata|null {
  const offset = this.bb!.__offset(this.bb_pos, 4);
  return offset ? (obj || new MsgMetadata()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

p0(obj?:Point2D):Point2D|null {
  const offset = this.bb!.__offset(this.bb_pos, 6);
  return offset ? (obj || new Point2D()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

p1(obj?:Point2D):Point2D|null {
  const offset = this.bb!.__offset(this.bb_pos, 8);
  return offset ? (obj || new Point2D()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

color():number {
  const offset = this.bb!.__offset(this.bb_pos, 10);
  return offset ? this.bb!.readUint32(this.bb_pos + offset) : 0;
}

static startColoredLine2D(builder:flatbuffers.Builder) {
  builder.startObject(4);
}

static add_Metadata(builder:flatbuffers.Builder, _MetadataOffset:flatbuffers.Offset) {
  builder.addFieldOffset(0, _MetadataOffset, 0);
}

static addP0(builder:flatbuffers.Builder, p0Offset:flatbuffers.Offset) {
  builder.addFieldOffset(1, p0Offset, 0);
}

static addP1(builder:flatbuffers.Builder, p1Offset:flatbuffers.Offset) {
  builder.addFieldOffset(2, p1Offset, 0);
}

static addColor(builder:flatbuffers.Builder, color:number) {
  builder.addFieldInt32(3, color, 0);
}

static endColoredLine2D(builder:flatbuffers.Builder):flatbuffers.Offset {
  const offset = builder.endObject();
  builder.requiredField(offset, 6) // p0
  builder.requiredField(offset, 8) // p1
  return offset;
}

}
