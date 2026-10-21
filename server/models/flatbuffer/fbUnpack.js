import { readdirSync, readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import * as fb from 'fbmsglib';
import { logger } from '../../common/logger.js';

// flatc writes a snake_case field (`frame_id`) as camelCase in the generated code (`frameId`),
// while a ROS message (rosbridge JSON, which every decoder in msgDecode expects) keeps the name it
// has in the .msg. A field that is already camelCase in the .fbs (DJI's `curWaypointIndex`) stays
// as it is, so camel→snake cannot be inverted by guessing: the original names are read from the
// .fbs themselves, which are the source of truth. No two different names share a camelCase form
// (checked in test/test-fb-via-category.js).
// flatc lowercases an ALL_CAPS name first (HEALT_WORKING → healtWorking).
const camel = (s) => (s === s.toUpperCase() ? s.toLowerCase() : s).replace(/_([a-z0-9])/g, (_, c) => c.toUpperCase());
// ROS constants are UPPER_CASE. schema_dji_osdk_ros.fbs declares some INSIDE a table, which flatc
// turns into real fields (with their default value): they never travel in a ROS message.
const isConstant = (name) => /[A-Z]/.test(name) && name === name.toUpperCase();
const snake = (s) => s.replace(/[A-Z]/g, (c) => '_' + c.toLowerCase());

function loadRosFieldNames() {
  const names = new Map(); // generated key → name in the .msg
  try {
    const dir = join(dirname(fileURLToPath(import.meta.resolve('fbmsglib'))), '..', 'messages');
    for (const file of readdirSync(dir).filter((f) => f.endsWith('.fbs'))) {
      for (const line of readFileSync(join(dir, file), 'utf8').split('\n')) {
        const field = line.match(/^\s*([A-Za-z][A-Za-z0-9_]*)\s*:\s*[[\w.]/)?.[1];
        if (field) names.set(camel(field), field);
      }
    }
  } catch (err) {
    logger.warn(`fbUnpack: could not read the fbmsglib schemas (${err.message}); falling back to snake_case`);
  }
  return names;
}

const rosFieldNames = loadRosFieldNames();

/** The .msg name of a generated field key, or null when the schemas do not declare it. */
export const rosFieldName = (key) => rosFieldNames.get(key) ?? null;

/** unpack() output → the shape rosbridge would deliver: .msg field names, no robofleet metadata. */
export function rosShape(value) {
  if (Array.isArray(value)) return value.map(rosShape);
  if (value === null || typeof value !== 'object' || ArrayBuffer.isView(value)) return value;
  const out = {};
  for (const [key, v] of Object.entries(value)) {
    if (key === '_Metadata') continue;
    const name = rosFieldName(key);
    if (name && isConstant(name)) continue;
    out[name ?? snake(key)] = rosShape(v);
  }
  return out;
}

// Longest byte vector (CompressedImage.data, PointCloud2.data) shown as numbers; longer ones
// are summarized so one image does not turn into a multi-MB JSON.
const MAX_BYTES_SHOWN = 64;
// Bytes of an unsupported message shown as hex.
const HEX_BYTES = 256;

// type → { cls, name } | null. The ROS type is "<package>/<Message>" and fbmsglib mirrors that
// exactly (namespace = package, table = message), so no table of types is kept: a new .fbs is
// inspectable as soon as the lib is regenerated. A ROS2 type ("pkg/msg/Name") resolves too.
const resolved = new Map();

function resolveTable(type) {
  if (resolved.has(type)) return resolved.get(type);
  const parts = String(type).split('/');
  const pkg = parts[0];
  const name = parts.at(-1);
  const ns = Object.hasOwn(fb.fb, pkg) ? fb.fb[pkg] : null;
  const cls = ns && Object.hasOwn(ns, name) ? ns[name] : null;
  const table = typeof cls?.[`getRootAs${name}`] === 'function' ? { cls, name } : null;
  resolved.set(type, table);
  return table;
}

function jsonReplacer(_key, value) {
  if (ArrayBuffer.isView(value) && !(value instanceof DataView)) {
    return value.length > MAX_BYTES_SHOWN ? `<${value.length} items>` : Array.from(value);
  }
  return typeof value === 'bigint' ? value.toString() : value;
}

/** unpack() output → plain JSON-safe object (typed arrays summarized, bigint as string). */
export function toJsonSafe(obj) {
  return JSON.parse(JSON.stringify(obj, jsonReplacer));
}

function hexView(buf) {
  const bytes = buf.bytes();
  return { size: bytes.length, hex: Buffer.from(bytes.subarray(0, HEX_BYTES)).toString('hex') };
}

/**
 * A robofleet message as the ROS message it carries (what rosbridge would have delivered), or
 * null when fbmsglib has no table for `type`. Unpacks EVERY field; throws on a malformed buffer.
 * @param {string} type ROS message type from the robofleet metadata
 * @param {import('flatbuffers').ByteBuffer} buf
 */
export function fbMsgAsRos(type, buf) {
  const table = resolveTable(type);
  if (!table) return null;
  return rosShape(table.cls[`getRootAs${table.name}`](buf).unpack());
}

/**
 * The same message for the raw inspector: JSON-safe, and a hex view when it cannot be unpacked.
 * It shows what `decodeMsg` is going to receive.
 */
export function unpackFbMsg(type, buf) {
  try {
    const msg = fbMsgAsRos(type, buf);
    return msg ? toJsonSafe(msg) : { unsupported: String(type), ...hexView(buf) };
  } catch (err) {
    return { error: err.message, type, ...hexView(buf) };
  }
}
