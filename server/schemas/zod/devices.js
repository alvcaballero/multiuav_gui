import { z } from 'zod';
import { DEVICE_STATUS as DEVICE_STATUS_MAP } from '../../config/status.js';

export const DEVICE_STATUS = z.enum(Object.values(DEVICE_STATUS_MAP));
export const DEVICE_PROTOCOL = z.enum(['ros', 'robofleet', 'mqtt', 'mavlink']);

// `host` is where the camera's stream is served: the RTSP host MediaMTX pulls from (WebRTC) or the
// host the browser connects to directly (WebRTC_env). A Websocket camera arrives through the GCS
// (ROS/MQTT frames), so it has none. It lives here, not on the device: one robot can have cameras on
// different hosts, and a robot without cameras has no use for an address.
export const CameraSchema = z
  .object({
    type: z.enum(['WebRTC', 'WebRTC_env', 'RTSP', 'Websocket']),
    source: z.string(),
    host: z.string().trim().min(1).optional(),
  })
  .superRefine((camera, ctx) => {
    if (camera.type !== 'Websocket' && !camera.host) {
      ctx.addIssue({ code: z.ZodIssueCode.custom, path: ['host'], message: `a ${camera.type} camera needs a host` });
    }
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

// Per-device settings stored in Device.attributes. They are declared as GROUPS: a group says WHEN it
// applies (`appliesTo`, over the device's protocol, category or even another attribute) and which keys
// it adds (`shape`, zod fields). A device's attributes are the union of the groups that apply to it,
// strict: a key no applicable group declares is an error instead of a silent typo. A new per-robot
// parameter is a new group HERE and nothing else on the schema side.
export const ATTRIBUTE_GROUPS = [
  {
    id: 'mavlink',
    appliesTo: ({ protocol }) => protocol === 'mavlink',
    shape: {
      // MAVLink system id of the vehicle (MAV_SYS_ID on PX4, MAV_SYSID on ArduPilot >= 4.7). 0 is
      // broadcast and 255 is the GCS itself, so neither can name a vehicle. The vehicle is addressed
      // only by it: a MAVLink device without one could never be reached.
      systemId: z.coerce.number().int().min(1).max(254),
    },
  },
  {
    id: 'ros',
    appliesTo: ({ protocol }) => protocol === 'ros',
    shape: {
      // The companion runs the slow relay: the telemetry (subscribers only) is read from
      // `/<device>/slow<topic>`, republished at a lower rate, instead of `/<device><topic>`. A
      // deployment of THIS unit, not of its interface: any ROS interface works with or without it.
      // MQTT and robofleet already bring their telemetry at a lower rate.
      slow: z.boolean().optional(),
    },
  },
  {
    id: 'capabilities',
    appliesTo: () => true,
    shape: {
      // Switches this unit's capabilities against its model's (config/devices/capabilities.yaml):
      // `{ rtk: true }` enables an optional one (the module is fitted), `{ gimbal: false }` disables a
      // standard one (it is broken). Only the ones the model declares: checked by the controller
      // (deviceModelsModel.capabilityOverrideProblem), since schemas do not import models.
      capabilities: z.record(z.string(), z.boolean()).optional(),
    },
  },
  {
    id: 'radio',
    appliesTo: () => true,
    shape: {
      // The radio on board (a Ubiquiti station) whose link to the GCS's access point is watched
      // (models/radio/radioLinkMonitor.js). Either one finds it in the AP's station table: the IP is the
      // radio's own, NOT the companion computer's; the MAC survives an address change, so it wins when both
      // are given. null clears it (a PUT merges, it never drops a key by omission).
      radioIp: z.string().trim().ip({ version: 'v4' }).nullish(),
      radioMac: z
        .string()
        .trim()
        .regex(/^([0-9a-f]{2}[:-]){5}[0-9a-f]{2}$/i, 'a MAC address like aa:bb:cc:dd:ee:ff')
        .transform((mac) => mac.toLowerCase().replaceAll('-', ':'))
        .nullish(),
    },
  },
  {
    id: 'companion',
    appliesTo: () => true,
    shape: {
      // The onboard computer, pinged from the GCS to measure the real path to it (models/radio/hostPingMonitor.js).
      // Not the device's connection address (a device has none): this only feeds the link quality. null clears it.
      companionIp: z.string().trim().ip({ version: 'v4' }).nullish(),
    },
  },
];

/**
 * The zod fields of every group that applies to `device` ({ protocol, category, attributes }).
 * Two groups declaring the same key for one device is a programming error and throws.
 */
export function resolveAttributeShape(groups, device) {
  const shape = {};
  for (const group of groups) {
    if (!group.appliesTo({ ...device, protocol: device.protocol ?? 'ros', attributes: device.attributes ?? {} })) continue;
    for (const key of Object.keys(group.shape)) {
      if (key in shape) throw new Error(`attribute '${key}' is declared twice for one device (group '${group.id}')`);
      shape[key] = group.shape[key];
    }
  }
  return shape;
}

/**
 * Validates `attributes` against the groups that apply to `device` ({ protocol, category }).
 * @returns {import('zod').SafeParseReturnType} issue paths are relative to `attributes`
 */
export function validateDeviceAttributes(device, attributes) {
  const candidate = attributes ?? {};
  return z.object(resolveAttributeShape(ATTRIBUTE_GROUPS, { ...device, attributes: candidate })).strict().safeParse(candidate);
}

/** `attributes` without the keys no group applies to `device` any more (after a protocol/category change). */
export function pruneAttributes(device, attributes) {
  const allowed = resolveAttributeShape(ATTRIBUTE_GROUPS, { ...device, attributes });
  return Object.fromEntries(Object.entries(attributes ?? {}).filter(([key]) => key in allowed));
}

export const DeviceSchema = z.object({
  id: z.coerce.string().optional(),
  name: z.string(),
  category: z.string(),
  // config/devices/device_models/<key>.yaml. Optional (null = not chosen): a device created without it gets its category's default.
  deviceModel: z.string().nullish(),
  protocol: DEVICE_PROTOCOL.default('ros'),
  // Shape depends on the device (protocol, category…): checked by validateDevice / the controller, see ATTRIBUTE_GROUPS.
  attributes: z.record(z.unknown()).nullable().optional(),
  camera: z.array(CameraSchema).optional(),
  files: z.array(FileAccessSchema).optional(),
  lastUpdate: z.coerce.date().optional(),
  status: DEVICE_STATUS.default(DEVICE_STATUS_MAP.OFFLINE).optional(),
});

export const DevicePublicSchema = DeviceSchema.pick({
  id: true,
  name: true,
  category: true,
  deviceModel: true,
  camera: true,
  status: true,
  protocol: true,
  attributes: true,
  lastUpdate: true,
});

export const DevicePrivateSchema = DeviceSchema.pick({
  id: true,
  name: true,
  files: true,
});

// Replaces `attributes` with the version parsed by the groups that apply to the device (coerced,
// defaults applied); its issues are reported under the `attributes` path.
const withDeviceAttributes = (device, ctx) => {
  const parsed = validateDeviceAttributes(device, device.attributes);
  if (!parsed.success) {
    for (const issue of parsed.error.issues) ctx.addIssue({ ...issue, path: ['attributes', ...issue.path] });
    return z.NEVER;
  }
  return { ...device, attributes: parsed.data };
};

export function validateDevice(input) {
  return DeviceSchema.transform(withDeviceAttributes).safeParse(input);
}

export function validatePartialDevice(input) {
  return DeviceSchema.partial().safeParse(input);
}
