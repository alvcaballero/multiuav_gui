import { z } from 'zod';
import { DEVICE_STATUS as DEVICE_STATUS_MAP } from '../../config/status.js';

export const DEVICE_STATUS = z.enum(Object.values(DEVICE_STATUS_MAP));
export const DEVICE_PROTOCOL = z.enum(['ros', 'robofleet', 'mqtt', 'mavlink']);

export const CameraSchema = z.object({
  type: z.enum(['WebRTC', 'RTSP', 'Websocket']),
  source: z.string(),
});

// `type` selects a server preset (devices.yaml → files.<type>). The optional
// fields below override that preset per-device, field-by-field; omit one to keep
// the preset value. `downloadType` maps onto the preset's folder mode (`type`).
export const FileAccessSchema = z.object({
  url: z.string().url(),
  type: z.enum(['onboard_computer', 'wiris_pro', 'default']),
  path: z.string().optional(),
  downloadType: z.enum(['all', 'lastFolder', 'specific']).optional(),
  delete: z.boolean().optional(),
  srvDownload: z.boolean().optional(),
});

export const DeviceSchema = z.object({
  id: z.coerce.string().optional(),
  name: z.string(),
  category: z.string(),
  ip: z.string().ip(),
  protocol: DEVICE_PROTOCOL.default('ros'),
  // MAVLink system id of the vehicle (MAV_SYS_ID on PX4, MAV_SYSID on ArduPilot >= 4.7). 0 is
  // broadcast and 255 is the GCS itself, so neither can name a vehicle.
  systemId: z.coerce.number().int().min(1).max(254).nullable().optional(),
  camera: z.array(CameraSchema).optional(),
  files: z.array(FileAccessSchema).optional(),
  lastUpdate: z.coerce.date().optional(),
  status: DEVICE_STATUS.default(DEVICE_STATUS_MAP.OFFLINE).optional(),
  user: z.string().nullable().optional(),
  pwd: z.string().nullable().optional(),
});

export const DevicePublicSchema = DeviceSchema.pick({
  id: true,
  name: true,
  category: true,
  camera: true,
  status: true,
  protocol: true,
  systemId: true,
  lastUpdate: true,
});

export const DevicePrivateSchema = DeviceSchema.pick({
  id: true,
  name: true,
  user: true,
  pwd: true,
  ip: true,
  files: true,
});

// The vehicle is addressed only by its system id: a MAVLink device without one could never be
// reached, and its telemetry would never be matched to it.
const requireSystemIdForMavlink = (device, ctx) => {
  if (device.protocol === 'mavlink' && device.systemId == null) {
    ctx.addIssue({ code: z.ZodIssueCode.custom, path: ['systemId'], message: 'a mavlink device needs a systemId' });
  }
};

export function validateDevice(input) {
  return DeviceSchema.superRefine(requireSystemIdForMavlink).safeParse(input);
}

export function validatePartialDevice(input) {
  return DeviceSchema.partial().safeParse(input);
}
