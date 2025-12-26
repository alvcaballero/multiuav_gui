import { z } from 'zod';

export const DEVICE_STATUS = z.enum(['online', 'offline']);
export const DEVICE_PROTOCOL = z.enum(['ros', 'robofleet']);

export const CameraSchema = z.object({
  type: z.enum(['WebRTC', 'RTSP', 'Websocket']),
  source: z.string(),
});

export const FileAccessSchema = z.object({
  url: z.string().url(),
  type: z.literal('ftp'),
});

export const DeviceSchema = z.object({
  id: z.coerce.string().optional(),
  name: z.string(),
  category: z.string(),
  ip: z.string().ip(),
  protocol: DEVICE_PROTOCOL.default('ros'),
  camera: z.array(CameraSchema).optional(),
  files: z.array(FileAccessSchema).optional(),
  lastUpdate: z.coerce.date().optional(),
  status: DEVICE_STATUS.default('offline').optional(),
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

export function validateDevice(input) {
  return DeviceSchema.safeParse(input);
}

export function validatePartialDevice(input) {
  return DeviceSchema.partial().safeParse(input);
}
