// automatically generated by the FlatBuffers compiler, do not modify

/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */

import * as flatbuffers from 'flatbuffers';

import { MsgMetadata } from '../../fb/msg-metadata.js';
import { Header } from '../../fb/std-msgs/header.js';


export class BatteryState {
  bb: flatbuffers.ByteBuffer|null = null;
  bb_pos = 0;
  __init(i:number, bb:flatbuffers.ByteBuffer):BatteryState {
  this.bb_pos = i;
  this.bb = bb;
  return this;
}

static getRootAsBatteryState(bb:flatbuffers.ByteBuffer, obj?:BatteryState):BatteryState {
  return (obj || new BatteryState()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

static getSizePrefixedRootAsBatteryState(bb:flatbuffers.ByteBuffer, obj?:BatteryState):BatteryState {
  bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
  return (obj || new BatteryState()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

_Metadata(obj?:MsgMetadata):MsgMetadata|null {
  const offset = this.bb!.__offset(this.bb_pos, 4);
  return offset ? (obj || new MsgMetadata()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

header(obj?:Header):Header|null {
  const offset = this.bb!.__offset(this.bb_pos, 6);
  return offset ? (obj || new Header()).__init(this.bb!.__indirect(this.bb_pos + offset), this.bb!) : null;
}

voltage():number {
  const offset = this.bb!.__offset(this.bb_pos, 8);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

current():number {
  const offset = this.bb!.__offset(this.bb_pos, 10);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

charge():number {
  const offset = this.bb!.__offset(this.bb_pos, 12);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

capacity():number {
  const offset = this.bb!.__offset(this.bb_pos, 14);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

designCapacity():number {
  const offset = this.bb!.__offset(this.bb_pos, 16);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

percentage():number {
  const offset = this.bb!.__offset(this.bb_pos, 18);
  return offset ? this.bb!.readFloat32(this.bb_pos + offset) : 0.0;
}

powerSupplyStatus():number {
  const offset = this.bb!.__offset(this.bb_pos, 20);
  return offset ? this.bb!.readInt8(this.bb_pos + offset) : 0;
}

powerSupplyHealth():number {
  const offset = this.bb!.__offset(this.bb_pos, 22);
  return offset ? this.bb!.readInt8(this.bb_pos + offset) : 0;
}

powerSupplyTechnology():number {
  const offset = this.bb!.__offset(this.bb_pos, 24);
  return offset ? this.bb!.readInt8(this.bb_pos + offset) : 0;
}

present():boolean {
  const offset = this.bb!.__offset(this.bb_pos, 26);
  return offset ? !!this.bb!.readInt8(this.bb_pos + offset) : false;
}

cellVoltage(index: number):number|null {
  const offset = this.bb!.__offset(this.bb_pos, 28);
  return offset ? this.bb!.readFloat32(this.bb!.__vector(this.bb_pos + offset) + index * 4) : 0;
}

cellVoltageLength():number {
  const offset = this.bb!.__offset(this.bb_pos, 28);
  return offset ? this.bb!.__vector_len(this.bb_pos + offset) : 0;
}

cellVoltageArray():Float32Array|null {
  const offset = this.bb!.__offset(this.bb_pos, 28);
  return offset ? new Float32Array(this.bb!.bytes().buffer, this.bb!.bytes().byteOffset + this.bb!.__vector(this.bb_pos + offset), this.bb!.__vector_len(this.bb_pos + offset)) : null;
}

cellTemperature(index: number):number|null {
  const offset = this.bb!.__offset(this.bb_pos, 30);
  return offset ? this.bb!.readFloat32(this.bb!.__vector(this.bb_pos + offset) + index * 4) : 0;
}

cellTemperatureLength():number {
  const offset = this.bb!.__offset(this.bb_pos, 30);
  return offset ? this.bb!.__vector_len(this.bb_pos + offset) : 0;
}

cellTemperatureArray():Float32Array|null {
  const offset = this.bb!.__offset(this.bb_pos, 30);
  return offset ? new Float32Array(this.bb!.bytes().buffer, this.bb!.bytes().byteOffset + this.bb!.__vector(this.bb_pos + offset), this.bb!.__vector_len(this.bb_pos + offset)) : null;
}

static startBatteryState(builder:flatbuffers.Builder) {
  builder.startObject(14);
}

static add_Metadata(builder:flatbuffers.Builder, _MetadataOffset:flatbuffers.Offset) {
  builder.addFieldOffset(0, _MetadataOffset, 0);
}

static addHeader(builder:flatbuffers.Builder, headerOffset:flatbuffers.Offset) {
  builder.addFieldOffset(1, headerOffset, 0);
}

static addVoltage(builder:flatbuffers.Builder, voltage:number) {
  builder.addFieldFloat32(2, voltage, 0.0);
}

static addCurrent(builder:flatbuffers.Builder, current:number) {
  builder.addFieldFloat32(3, current, 0.0);
}

static addCharge(builder:flatbuffers.Builder, charge:number) {
  builder.addFieldFloat32(4, charge, 0.0);
}

static addCapacity(builder:flatbuffers.Builder, capacity:number) {
  builder.addFieldFloat32(5, capacity, 0.0);
}

static addDesignCapacity(builder:flatbuffers.Builder, designCapacity:number) {
  builder.addFieldFloat32(6, designCapacity, 0.0);
}

static addPercentage(builder:flatbuffers.Builder, percentage:number) {
  builder.addFieldFloat32(7, percentage, 0.0);
}

static addPowerSupplyStatus(builder:flatbuffers.Builder, powerSupplyStatus:number) {
  builder.addFieldInt8(8, powerSupplyStatus, 0);
}

static addPowerSupplyHealth(builder:flatbuffers.Builder, powerSupplyHealth:number) {
  builder.addFieldInt8(9, powerSupplyHealth, 0);
}

static addPowerSupplyTechnology(builder:flatbuffers.Builder, powerSupplyTechnology:number) {
  builder.addFieldInt8(10, powerSupplyTechnology, 0);
}

static addPresent(builder:flatbuffers.Builder, present:boolean) {
  builder.addFieldInt8(11, +present, +false);
}

static addCellVoltage(builder:flatbuffers.Builder, cellVoltageOffset:flatbuffers.Offset) {
  builder.addFieldOffset(12, cellVoltageOffset, 0);
}

static createCellVoltageVector(builder:flatbuffers.Builder, data:number[]|Float32Array):flatbuffers.Offset;
/**
 * @deprecated This Uint8Array overload will be removed in the future.
 */
static createCellVoltageVector(builder:flatbuffers.Builder, data:number[]|Uint8Array):flatbuffers.Offset;
static createCellVoltageVector(builder:flatbuffers.Builder, data:number[]|Float32Array|Uint8Array):flatbuffers.Offset {
  builder.startVector(4, data.length, 4);
  for (let i = data.length - 1; i >= 0; i--) {
    builder.addFloat32(data[i]!);
  }
  return builder.endVector();
}

static startCellVoltageVector(builder:flatbuffers.Builder, numElems:number) {
  builder.startVector(4, numElems, 4);
}

static addCellTemperature(builder:flatbuffers.Builder, cellTemperatureOffset:flatbuffers.Offset) {
  builder.addFieldOffset(13, cellTemperatureOffset, 0);
}

static createCellTemperatureVector(builder:flatbuffers.Builder, data:number[]|Float32Array):flatbuffers.Offset;
/**
 * @deprecated This Uint8Array overload will be removed in the future.
 */
static createCellTemperatureVector(builder:flatbuffers.Builder, data:number[]|Uint8Array):flatbuffers.Offset;
static createCellTemperatureVector(builder:flatbuffers.Builder, data:number[]|Float32Array|Uint8Array):flatbuffers.Offset {
  builder.startVector(4, data.length, 4);
  for (let i = data.length - 1; i >= 0; i--) {
    builder.addFloat32(data[i]!);
  }
  return builder.endVector();
}

static startCellTemperatureVector(builder:flatbuffers.Builder, numElems:number) {
  builder.startVector(4, numElems, 4);
}

static endBatteryState(builder:flatbuffers.Builder):flatbuffers.Offset {
  const offset = builder.endObject();
  builder.requiredField(offset, 6) // header
  builder.requiredField(offset, 28) // cell_voltage
  builder.requiredField(offset, 30) // cell_temperature
  return offset;
}

}