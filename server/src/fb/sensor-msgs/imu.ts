// automatically generated by the FlatBuffers compiler, do not modify

/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */

import * as flatbuffers from 'flatbuffers';

import { MsgMetadata } from '../../fb/msg-metadata.js';
import { Quaternion } from '../../fb/geometry-msgs/quaternion.js';
import { Vector3 } from '../../fb/geometry-msgs/vector3.js';
import { Header } from '../../fb/std-msgs/header.js';


export class Imu {
  bb: flatbuffers.ByteBuffer|null = null;
  bb_pos = 0;
  __init(i:number, bb:flatbuffers.ByteBuffer):Imu {
  this.bb_pos = i;
  this.bb = bb;
  return this;
}

static getRootAsImu(bb:flatbuffers.ByteBuffer, obj?:Imu):Imu {
  return (obj || new Imu()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

static getSizePrefixedRootAsImu(bb:flatbuffers.ByteBuffer, obj?:Imu):Imu {
  bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
  return (obj || new Imu()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

_Metadata(obj?:MsgMetadata):MsgMetadata|null {
  const offset = this.bb!.__offset(this.bb_pos, 4);
  return offset ? (obj || new MsgMetadata()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

header(obj?:Header):Header|null {
  const offset = this.bb!.__offset(this.bb_pos, 6);
  return offset ? (obj || new Header()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

orientation(obj?:Quaternion):Quaternion|null {
  const offset = this.bb!.__offset(this.bb_pos, 8);
  return offset ? (obj || new Quaternion()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

orientationCovariance(index: number):number|null {
  const offset = this.bb!.__offset(this.bb_pos, 10);
  return offset ? this.bb!.readFloat64(this.bb!.__vector(this.bb_pos + offset) + index * 8) : 0;
}

orientationCovarianceLength():number {
  const offset = this.bb!.__offset(this.bb_pos, 10);
  return offset ? this.bb!.__vector_len(this.bb_pos + offset) : 0;
}

orientationCovarianceArray():Float64Array|null {
  const offset = this.bb!.__offset(this.bb_pos, 10);
  return offset ? new Float64Array(this.bb!.bytes().buffer, this.bb!.bytes().byteOffset + this.bb!.__vector(this.bb_pos + offset), this.bb!.__vector_len(this.bb_pos + offset)) : null;
}

angularVelocity(obj?:Vector3):Vector3|null {
  const offset = this.bb!.__offset(this.bb_pos, 12);
  return offset ? (obj || new Vector3()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

angularVelocityCovariance(index: number):number|null {
  const offset = this.bb!.__offset(this.bb_pos, 14);
  return offset ? this.bb!.readFloat64(this.bb!.__vector(this.bb_pos + offset) + index * 8) : 0;
}

angularVelocityCovarianceLength():number {
  const offset = this.bb!.__offset(this.bb_pos, 14);
  return offset ? this.bb!.__vector_len(this.bb_pos + offset) : 0;
}

angularVelocityCovarianceArray():Float64Array|null {
  const offset = this.bb!.__offset(this.bb_pos, 14);
  return offset ? new Float64Array(this.bb!.bytes().buffer, this.bb!.bytes().byteOffset + this.bb!.__vector(this.bb_pos + offset), this.bb!.__vector_len(this.bb_pos + offset)) : null;
}

linearAcceleration(obj?:Vector3):Vector3|null {
  const offset = this.bb!.__offset(this.bb_pos, 16);
  return offset ? (obj || new Vector3()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

linearAccelerationCovariance(index: number):number|null {
  const offset = this.bb!.__offset(this.bb_pos, 18);
  return offset ? this.bb!.readFloat64(this.bb!.__vector(this.bb_pos + offset) + index * 8) : 0;
}

linearAccelerationCovarianceLength():number {
  const offset = this.bb!.__offset(this.bb_pos, 18);
  return offset ? this.bb!.__vector_len(this.bb_pos + offset) : 0;
}

linearAccelerationCovarianceArray():Float64Array|null {
  const offset = this.bb!.__offset(this.bb_pos, 18);
  return offset ? new Float64Array(this.bb!.bytes().buffer, this.bb!.bytes().byteOffset + this.bb!.__vector(this.bb_pos + offset), this.bb!.__vector_len(this.bb_pos + offset)) : null;
}

static startImu(builder:flatbuffers.Builder) {
  builder.startObject(8);
}

static add_Metadata(builder:flatbuffers.Builder, _MetadataOffset:flatbuffers.Offset) {
  builder.addFieldOffset(0, _MetadataOffset, 0);
}

static addHeader(builder:flatbuffers.Builder, headerOffset:flatbuffers.Offset) {
  builder.addFieldOffset(1, headerOffset, 0);
}

static addOrientation(builder:flatbuffers.Builder, orientationOffset:flatbuffers.Offset) {
  builder.addFieldOffset(2, orientationOffset, 0);
}

static addOrientationCovariance(builder:flatbuffers.Builder, orientationCovarianceOffset:flatbuffers.Offset) {
  builder.addFieldOffset(3, orientationCovarianceOffset, 0);
}

static createOrientationCovarianceVector(builder:flatbuffers.Builder, data:number[]|Float64Array):flatbuffers.Offset;
/**
 * @deprecated This Uint8Array overload will be removed in the future.
 */
static createOrientationCovarianceVector(builder:flatbuffers.Builder, data:number[]|Uint8Array):flatbuffers.Offset;
static createOrientationCovarianceVector(builder:flatbuffers.Builder, data:number[]|Float64Array|Uint8Array):flatbuffers.Offset {
  builder.startVector(8, data.length, 8);
  for (let i = data.length - 1; i >= 0; i--) {
    builder.addFloat64(data[i]!);
  }
  return builder.endVector();
}

static startOrientationCovarianceVector(builder:flatbuffers.Builder, numElems:number) {
  builder.startVector(8, numElems, 8);
}

static addAngularVelocity(builder:flatbuffers.Builder, angularVelocityOffset:flatbuffers.Offset) {
  builder.addFieldOffset(4, angularVelocityOffset, 0);
}

static addAngularVelocityCovariance(builder:flatbuffers.Builder, angularVelocityCovarianceOffset:flatbuffers.Offset) {
  builder.addFieldOffset(5, angularVelocityCovarianceOffset, 0);
}

static createAngularVelocityCovarianceVector(builder:flatbuffers.Builder, data:number[]|Float64Array):flatbuffers.Offset;
/**
 * @deprecated This Uint8Array overload will be removed in the future.
 */
static createAngularVelocityCovarianceVector(builder:flatbuffers.Builder, data:number[]|Uint8Array):flatbuffers.Offset;
static createAngularVelocityCovarianceVector(builder:flatbuffers.Builder, data:number[]|Float64Array|Uint8Array):flatbuffers.Offset {
  builder.startVector(8, data.length, 8);
  for (let i = data.length - 1; i >= 0; i--) {
    builder.addFloat64(data[i]!);
  }
  return builder.endVector();
}

static startAngularVelocityCovarianceVector(builder:flatbuffers.Builder, numElems:number) {
  builder.startVector(8, numElems, 8);
}

static addLinearAcceleration(builder:flatbuffers.Builder, linearAccelerationOffset:flatbuffers.Offset) {
  builder.addFieldOffset(6, linearAccelerationOffset, 0);
}

static addLinearAccelerationCovariance(builder:flatbuffers.Builder, linearAccelerationCovarianceOffset:flatbuffers.Offset) {
  builder.addFieldOffset(7, linearAccelerationCovarianceOffset, 0);
}

static createLinearAccelerationCovarianceVector(builder:flatbuffers.Builder, data:number[]|Float64Array):flatbuffers.Offset;
/**
 * @deprecated This Uint8Array overload will be removed in the future.
 */
static createLinearAccelerationCovarianceVector(builder:flatbuffers.Builder, data:number[]|Uint8Array):flatbuffers.Offset;
static createLinearAccelerationCovarianceVector(builder:flatbuffers.Builder, data:number[]|Float64Array|Uint8Array):flatbuffers.Offset {
  builder.startVector(8, data.length, 8);
  for (let i = data.length - 1; i >= 0; i--) {
    builder.addFloat64(data[i]!);
  }
  return builder.endVector();
}

static startLinearAccelerationCovarianceVector(builder:flatbuffers.Builder, numElems:number) {
  builder.startVector(8, numElems, 8);
}

static endImu(builder:flatbuffers.Builder):flatbuffers.Offset {
  const offset = builder.endObject();
  builder.requiredField(offset, 6) // header
  builder.requiredField(offset, 8) // orientation
  builder.requiredField(offset, 10) // orientation_covariance
  builder.requiredField(offset, 12) // angular_velocity
  builder.requiredField(offset, 14) // angular_velocity_covariance
  builder.requiredField(offset, 16) // linear_acceleration
  builder.requiredField(offset, 18) // linear_acceleration_covariance
  return offset;
}

}
