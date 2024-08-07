// automatically generated by the FlatBuffers compiler, do not modify

/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/no-non-null-assertion */

import * as flatbuffers from 'flatbuffers';

export class ConfigMissionResponse {
  bb: flatbuffers.ByteBuffer|null = null;
  bb_pos = 0;
  __init(i:number, bb:flatbuffers.ByteBuffer):ConfigMissionResponse {
  this.bb_pos = i;
  this.bb = bb;
  return this;
}

static getRootAsConfigMissionResponse(bb:flatbuffers.ByteBuffer, obj?:ConfigMissionResponse):ConfigMissionResponse {
  return (obj || new ConfigMissionResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

static getSizePrefixedRootAsConfigMissionResponse(bb:flatbuffers.ByteBuffer, obj?:ConfigMissionResponse):ConfigMissionResponse {
  bb.setPosition(bb.position() + flatbuffers.SIZE_PREFIX_LENGTH);
  return (obj || new ConfigMissionResponse()).__init(bb.readInt32(bb.position()) + bb.position(), bb);
}

success():boolean {
  const offset = this.bb!.__offset(this.bb_pos, 4);
  return offset ? !!this.bb!.readInt8(this.bb_pos + offset) : false;
}

static startConfigMissionResponse(builder:flatbuffers.Builder) {
  builder.startObject(1);
}

static addSuccess(builder:flatbuffers.Builder, success:boolean) {
  builder.addFieldInt8(0, +success, +false);
}

static endConfigMissionResponse(builder:flatbuffers.Builder):flatbuffers.Offset {
  const offset = builder.endObject();
  return offset;
}

static createConfigMissionResponse(builder:flatbuffers.Builder, success:boolean):flatbuffers.Offset {
  ConfigMissionResponse.startConfigMissionResponse(builder);
  ConfigMissionResponse.addSuccess(builder, success);
  return ConfigMissionResponse.endConfigMissionResponse(builder);
}
}
