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
exports.BatteryState = void 0;
/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */
const flatbuffers = __importStar(require('flatbuffers'));
const msg_metadata_js_1 = require('../../fb/msg-metadata.cjs');
const header_js_1 = require('../../fb/std-msgs/header.cjs');
class BatteryState {
  constructor() {
    this.bb = null;
    this.bb_pos = 0;
  }
  __init(i, bb) {
    this.bb_pos = i;
    this.bb = bb;
    return this;
  }
  static getRootAsBatteryState(bb, obj) {
    return (obj || new BatteryState()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
  }
  static getSizePrefixedRootAsBatteryState(bb, obj) {
    bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
    return (obj || new BatteryState()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
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
  voltage() {
    const offset = this.bb.__offset(this.bb_pos, 8);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  current() {
    const offset = this.bb.__offset(this.bb_pos, 10);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  charge() {
    const offset = this.bb.__offset(this.bb_pos, 12);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  capacity() {
    const offset = this.bb.__offset(this.bb_pos, 14);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  designCapacity() {
    const offset = this.bb.__offset(this.bb_pos, 16);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  percentage() {
    const offset = this.bb.__offset(this.bb_pos, 18);
    return offset ? this.bb.readFloat32(this.bb_pos + offset) : 0.0;
  }
  powerSupplyStatus() {
    const offset = this.bb.__offset(this.bb_pos, 20);
    return offset ? this.bb.readInt8(this.bb_pos + offset) : 0;
  }
  powerSupplyHealth() {
    const offset = this.bb.__offset(this.bb_pos, 22);
    return offset ? this.bb.readInt8(this.bb_pos + offset) : 0;
  }
  powerSupplyTechnology() {
    const offset = this.bb.__offset(this.bb_pos, 24);
    return offset ? this.bb.readInt8(this.bb_pos + offset) : 0;
  }
  present() {
    const offset = this.bb.__offset(this.bb_pos, 26);
    return offset ? !!this.bb.readInt8(this.bb_pos + offset) : false;
  }
  cellVoltage(index) {
    const offset = this.bb.__offset(this.bb_pos, 28);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  cellVoltageLength() {
    const offset = this.bb.__offset(this.bb_pos, 28);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  cellVoltageArray() {
    const offset = this.bb.__offset(this.bb_pos, 28);
    return offset
      ? new Float32Array(
          this.bb.bytes().buffer,
          this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset),
          this.bb.__vector_len(this.bb_pos + offset)
        )
      : null;
  }
  cellTemperature(index) {
    const offset = this.bb.__offset(this.bb_pos, 30);
    return offset ? this.bb.readFloat32(this.bb.__vector(this.bb_pos + offset) + index * 4) : 0;
  }
  cellTemperatureLength() {
    const offset = this.bb.__offset(this.bb_pos, 30);
    return offset ? this.bb.__vector_len(this.bb_pos + offset) : 0;
  }
  cellTemperatureArray() {
    const offset = this.bb.__offset(this.bb_pos, 30);
    return offset
      ? new Float32Array(
          this.bb.bytes().buffer,
          this.bb.bytes().byteOffset + this.bb.__vector(this.bb_pos + offset),
          this.bb.__vector_len(this.bb_pos + offset)
        )
      : null;
  }
  static startBatteryState(builder) {
    builder.startObject(14);
  }
  static add_Metadata(builder, _MetadataOffset) {
    builder.addFieldOffset(0, _MetadataOffset, 0);
  }
  static addHeader(builder, headerOffset) {
    builder.addFieldOffset(1, headerOffset, 0);
  }
  static addVoltage(builder, voltage) {
    builder.addFieldFloat32(2, voltage, 0.0);
  }
  static addCurrent(builder, current) {
    builder.addFieldFloat32(3, current, 0.0);
  }
  static addCharge(builder, charge) {
    builder.addFieldFloat32(4, charge, 0.0);
  }
  static addCapacity(builder, capacity) {
    builder.addFieldFloat32(5, capacity, 0.0);
  }
  static addDesignCapacity(builder, designCapacity) {
    builder.addFieldFloat32(6, designCapacity, 0.0);
  }
  static addPercentage(builder, percentage) {
    builder.addFieldFloat32(7, percentage, 0.0);
  }
  static addPowerSupplyStatus(builder, powerSupplyStatus) {
    builder.addFieldInt8(8, powerSupplyStatus, 0);
  }
  static addPowerSupplyHealth(builder, powerSupplyHealth) {
    builder.addFieldInt8(9, powerSupplyHealth, 0);
  }
  static addPowerSupplyTechnology(builder, powerSupplyTechnology) {
    builder.addFieldInt8(10, powerSupplyTechnology, 0);
  }
  static addPresent(builder, present) {
    builder.addFieldInt8(11, +present, +false);
  }
  static addCellVoltage(builder, cellVoltageOffset) {
    builder.addFieldOffset(12, cellVoltageOffset, 0);
  }
  static createCellVoltageVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startCellVoltageVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static addCellTemperature(builder, cellTemperatureOffset) {
    builder.addFieldOffset(13, cellTemperatureOffset, 0);
  }
  static createCellTemperatureVector(builder, data) {
    builder.startVector(4, data.length, 4);
    for (let i = data.length - 1; i >= 0; i--) {
      builder.addFloat32(data[i]);
    }
    return builder.endVector();
  }
  static startCellTemperatureVector(builder, numElems) {
    builder.startVector(4, numElems, 4);
  }
  static endBatteryState(builder) {
    const offset = builder.endObject();
    builder.requiredField(offset, 6); // header
    builder.requiredField(offset, 28); // cell_voltage
    builder.requiredField(offset, 30); // cell_temperature
    return offset;
  }
}
exports.BatteryState = BatteryState;
