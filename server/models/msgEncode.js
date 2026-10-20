// Message encoding — domain object → the structure its msgType defines.
//
// The structure of a message depends ONLY on its msgType, and it is the same for every
// protocol: rosbridge JSON (ros), the same JSON over MQTT (mqtt), and robofleet, which packs
// ROS messages into Flatbuffers generated from the same .msg/.srv. So encoding happens ONCE,
// before the gateway (commandsModel.sendToDevice, and the raw /ros routes); a gateway only
// packs bytes.
//
// A registry by msgType, like rosDecode on the way in. What differs between categories is
// never a branch on the category: it is the msgType the category declares.
//
//   register(msgType, encode)  domain object → message
//   passthrough(msgType)       the domain object already IS the message (e.g. the PX4
//                              VehicleCommand that vehicleCommand.js builds)
//
// An unregistered msgType also goes out unchanged at runtime (the raw /ros routes publish any
// type), but the boot check requires every msgType a command endpoint uses to be registered,
// explicitly: a missing encoder there must not reach a UAV as a silently unencoded message.

import { MissionToRos, MissionToRos2 } from './mission/missionEncodeConfig.js';
import { MissionToPsdkV2 } from './mission/missionEncodePsdk.js';

const encoders = new Map(); // msgType → (msg) => message

function register(msgTypes, encode) {
  for (const msgType of [msgTypes].flat()) {
    if (encoders.has(msgType)) throw new Error(`msgEncode: duplicate encoder for ${msgType}`);
    encoders.set(msgType, encode);
  }
}

const identity = (msg) => msg;
const passthrough = (msgTypes) => register(msgTypes, identity);
const empty = () => ({});

/** True when `msgType` has an encoder, an explicit passthrough included. */
export function hasMsgEncoder(msgType) {
  return encoders.has(msgType);
}

/**
 * @param {string} msgType  the type the category declares for the endpoint
 * @param {object} [msg]    domain object (a command encoder's output, or a raw route body)
 * @returns {object} the message; `{}` when there is nothing to send
 */
export function encodeMsg(msgType, msg) {
  if (msg == null) return {};
  const encode = encoders.get(msgType);
  return encode ? encode(msg) : msg;
}

// Degrees → radians. The UI/domain works in degrees (like mission_schema); the
// PSDK gimbal message expects radians.
function deg2rad(deg) {
  return (deg * Math.PI) / 180;
}

// The download service requires UTC 0 timestamps in full ISO 8601 with the
// trailing `Z` (e.g. "2026-07-13T14:30:00.000Z"). Reject anything else early so
// a malformed/local-time date never reaches the UAV as a silent bad range.
const ISO_UTC_RE = /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d{3})?Z$/;
function assertUtcIsoDate(value, field) {
  if (typeof value !== 'string' || !ISO_UTC_RE.test(value) || Number.isNaN(Date.parse(value))) {
    throw new Error(
      `${field} must be a UTC ISO 8601 date ending in 'Z' (e.g. 2026-07-13T14:30:00.000Z), got: ${JSON.stringify(value)}`
    );
  }
  return value;
}

// ─── mission: task → configure_mission ────────────────────────────────────────
// Each encoder family lives whole in its own module (ConfigMission in missionEncodeConfig,
// PSDK in missionEncodePsdk); here they are only registered by msgType.
register(['aerialcore_common/ConfigMission', 'multiuav_interfaces/ConfigMission'], MissionToRos);
register('muav_gcs_interfaces/srv/LoadMission', MissionToRos2);
register('psdk_interfaces/srv/InitWaypointV2Setting', MissionToPsdkV2);

// ─── services with an empty request ───────────────────────────────────────────
register(
  [
    'std_srvs/TriggerRequest',
    'psdk_interfaces/srv/StartWaypointV2Mission',
    'psdk_interfaces/srv/StopWaypointV2Mission',
    'psdk_interfaces/srv/PauseWaypointV2Mission',
    'psdk_interfaces/srv/ResumeWaypointV2Mission',
  ],
  empty
);

// ─── gimbal ───────────────────────────────────────────────────────────────────
// Same NEUTRAL domain object for both: { pitch, roll, yaw, reset } in ABSOLUTE degrees.

// DJI OSDK (ROS1), dji_osdk_ros/GimbalAction.
register('dji_osdk_ros/GimbalAction', (msg) => ({
  header: { seq: 0, stamp: { secs: 0, nsecs: 0 }, frame_id: '' },
  is_reset: msg.reset ? true : false,
  payload_index: 0,
  // OSDK GimbalModule::Rotation: 0 = incremental over the current gimbal angle,
  // 1 = absolute (ground frame NEU). The domain is absolute → 1, same as PSDK.
  rotationMode: 1,
  pitch: msg.pitch ?? 0.0,
  roll: msg.roll ?? 0.0,
  yaw: msg.yaw ?? 0.0,
  time: 0.0,
}));

// DJI PSDK (ROS2), psdk_interfaces/msg/GimbalRotation: RADIANS and rotation_mode
// (0 = incremental, 1 = absolute ground, 2 = speed). Default 1 (absolute).
register('psdk_interfaces/msg/GimbalRotation', (msg) => ({
  payload_index: msg.payload_index || 1,
  rotation_mode: msg.rotation_mode ?? 1,
  pitch: deg2rad(msg.pitch ?? 0),
  roll: deg2rad(msg.roll ?? 0),
  yaw: deg2rad(msg.yaw ?? 0),
  time: 0.0,
}));

// ─── camera files ─────────────────────────────────────────────────────────────
register('muav_gcs_interfaces/action/DownloadFilesByDateRange', (msg) => ({
  payload_index: msg.payload_index || 1,
  init_date: assertUtcIsoDate(msg.startDate, 'init_date'),
  finish_date: assertUtcIsoDate(msg.endDate, 'finish_date'),
  file_type: msg.file_type || 'all',
  delete_after_download: msg.delete_after_download || false,
}));
register('dji_osdk_ros/DownloadMedia', (msg) => ({
  initDate: assertUtcIsoDate(msg.startDate, 'initDate'),
  FinishDate: assertUtcIsoDate(msg.endDate, 'finishDate'),
  downloadCnt: 0,
}));

// ─── passthrough: what the command encoder builds already IS the message ──────
passthrough([
  'px4_msgs/msg/VehicleCommand', // vehicleCommand.js builds the whole PX4 message
  'std_srvs/SetBool', // CommandMission → { data: true }
  'std_srvs/srv/SetBool',
  'std_srvs/Bool', // no request → {}
  'std_srvs/Trigger',
  'dji_osdk_ros/PauseWaypointV2Mission',
  'dji_osdk_ros/ResumeWaypointV2Mission',
  'dji_osdk_ros/StopWaypointV2Mission',
  'dji_osdk_ros/SetupCameraStream', // the validated attributes (command_schema) are its fields
  'dji_osdk_ros/FileList',
  'nav2_msgs/action/NavigateToPose',
  'psdk_interfaces/srv/UploadWaypointV2Mission',
]);

// ─── PX4 setpoints (raw /ros/publish, MCP agv tools) ──────────────────────────
register('px4_msgs/msg/TrajectorySetpoint', (msg) => ({
  timestamp: msg.timestamp || 0,
  position: msg.position || [0, 0, 0],
  velocity: msg.velocity || [0, 0, 0],
  acceleration: msg.acceleration || [0, 0, 0],
  jerk: msg.jerk || [0, 0, 0],
  yaw: msg.yaw || 0,
  yawspeed: msg.yawspeed || 0,
}));
register('geometry_msgs/Twist', (msg) => ({
  linear: msg.linear || { x: 0, y: 0, z: 0 },
  angular: msg.angular || { x: 0, y: 0, z: 0 },
}));
