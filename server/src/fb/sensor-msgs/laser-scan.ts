// automatically generated by the FlatBuffers compiler, do not modify

/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */

import * as flatbuffers from 'flatbuffers';

import { MsgMetadata } from '../../fb/msg-metadata.js';
import { Header } from '../../fb/std-msgs/header.js';


export class LaserScan {
  bb: flatbuffers.ByteBuffer|null = null;
  bb_pos = 0;
  __init(i:number, bb:flatbuffers.ByteBuffer):LaserScan {
  this.bb_pos = i;
  this.bb = bb;
  return this;
}

static getRootAsLaserScan(bb:flatbuffers.ByteBuffer, obj?:LaserScan):LaserScan {
  return (obj || new LaserScan()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

static getSizePrefixedRootAsLaserScan(bb:flatbuffers.ByteBuffer, obj?:LaserScan):LaserScan {
  bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
  return (obj || new LaserScan()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

_Metadata(obj?:MsgMetadata):MsgMetadata|null {
  const offset = this.bb!.__offset(this.bb_pos, 4);
  return offset ? (obj || new MsgMetadata()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

header(obj?:Header):Header|null {
  const offset = this.bb!.__offset(this.bb_pos, 6);
  return offset ? (obj || new Header()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

angleMin():number {
  const offset = this.bb!.__offset(this.bb_pos, 8);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

angleMax():number {
  const offset = this.bb!.__offset(this.bb_pos, 10);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

angleIncrement():number {
  const offset = this.bb!.__offset(this.bb_pos, 12);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

timeIncrement():number {
  const offset = this.bb!.__offset(this.bb_pos, 14);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

scanTime():number {
  const offset = this.bb!.__offset(this.bb_pos, 16);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

rangeMin():number {
  const offset = this.bb!.__offset(this.bb_pos, 18);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

rangeMax():number {
  const offset = this.bb!.__offset(this.bb_pos, 20);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

ranges(index: number):number|null {
  const offset = this.bb!.__offset(this.bb_pos, 22);
  return offset ? this.bb!.readFloat32(this.bb!.__vector(this.bb_pos + offset) + index * 4) : 0;
}

rangesLength():number {
  const offset = this.bb!.__offset(this.bb_pos, 22);
  return offset ? this.bb!.__vector_len(this.bb_pos + offset) : 0;
}

rangesArray():Float32Array|null {
  const offset = this.bb!.__offset(this.bb_pos, 22);
  return offset ? new Float32Array(this.bb!.bytes().buffer, this.bb!.bytes().byteOffset + this.bb!.__vector(this.bb_pos + offset), this.bb!.__vector_len(this.bb_pos + offset)) : null;
}

intensities(index: number):number|null {
  const offset = this.bb!.__offset(this.bb_pos, 24);
  return offset ? this.bb!.readFloat32(this.bb!.__vector(this.bb_pos + offset) + index * 4) : 0;
}

intensitiesLength():number {
  const offset = this.bb!.__offset(this.bb_pos, 24);
  return offset ? this.bb!.__vector_len(this.bb_pos + offset) : 0;
}

intensitiesArray():Float32Array|null {
  const offset = this.bb!.__offset(this.bb_pos, 24);
  return offset ? new Float32Array(this.bb!.bytes().buffer, this.bb!.bytes().byteOffset + this.bb!.__vector(this.bb_pos + offset), this.bb!.__vector_len(this.bb_pos + offset)) : null;
}

static startLaserScan(builder:flatbuffers.Builder) {
  builder.startObject(11);
}

static add_Metadata(builder:flatbuffers.Builder, _MetadataOffset:flatbuffers.Offset) {
  builder.addFieldOffset(0, _MetadataOffset, 0);
}

static addHeader(builder:flatbuffers.Builder, headerOffset:flatbuffers.Offset) {
  builder.addFieldOffset(1, headerOffset, 0);
}

static addAngleMin(builder:flatbuffers.Builder, angleMin:number) {
  builder.addFieldFloat32(2, angleMin, 0.0);
}

static addAngleMax(builder:flatbuffers.Builder, angleMax:number) {
  builder.addFieldFloat32(3, angleMax, 0.0);
}

static addAngleIncrement(builder:flatbuffers.Builder, angleIncrement:number) {
  builder.addFieldFloat32(4, angleIncrement, 0.0);
}

static addTimeIncrement(builder:flatbuffers.Builder, timeIncrement:number) {
  builder.addFieldFloat32(5, timeIncrement, 0.0);
}

static addScanTime(builder:flatbuffers.Builder, scanTime:number) {
  builder.addFieldFloat32(6, scanTime, 0.0);
}

static addRangeMin(builder:flatbuffers.Builder, rangeMin:number) {
  builder.addFieldFloat32(7, rangeMin, 0.0);
}

static addRangeMax(builder:flatbuffers.Builder, rangeMax:number) {
  builder.addFieldFloat32(8, rangeMax, 0.0);
}

static addRanges(builder:flatbuffers.Builder, rangesOffset:flatbuffers.Offset) {
  builder.addFieldOffset(9, rangesOffset, 0);
}

static createRangesVector(builder:flatbuffers.Builder, data:number[]|Float32Array):flatbuffers.Offset;
/**
 * @deprecated This Uint8Array overload will be removed in the future.
 */
static createRangesVector(builder:flatbuffers.Builder, data:number[]|Uint8Array):flatbuffers.Offset;
static createRangesVector(builder:flatbuffers.Builder, data:number[]|Float32Array|Uint8Array):flatbuffers.Offset {
  builder.startVector(4, data.length, 4);
  for (let i = data.length - 1; i >= 0; i--) {
    builder.addFloat32(data[i]!);
  }
  return builder.endVector();
}

static startRangesVector(builder:flatbuffers.Builder, numElems:number) {
  builder.startVector(4, numElems, 4);
}

static addIntensities(builder:flatbuffers.Builder, intensitiesOffset:flatbuffers.Offset) {
  builder.addFieldOffset(10, intensitiesOffset, 0);
}

static createIntensitiesVector(builder:flatbuffers.Builder, data:number[]|Float32Array):flatbuffers.Offset;
/**
 * @deprecated This Uint8Array overload will be removed in the future.
 */
static createIntensitiesVector(builder:flatbuffers.Builder, data:number[]|Uint8Array):flatbuffers.Offset;
static createIntensitiesVector(builder:flatbuffers.Builder, data:number[]|Float32Array|Uint8Array):flatbuffers.Offset {
  builder.startVector(4, data.length, 4);
  for (let i = data.length - 1; i >= 0; i--) {
    builder.addFloat32(data[i]!);
  }
  return builder.endVector();
}

static startIntensitiesVector(builder:flatbuffers.Builder, numElems:number) {
  builder.startVector(4, numElems, 4);
}

static endLaserScan(builder:flatbuffers.Builder):flatbuffers.Offset {
  const offset = builder.endObject();
  builder.requiredField(offset, 6) // header
  builder.requiredField(offset, 22) // ranges
  builder.requiredField(offset, 24) // intensities
  return offset;
}

}