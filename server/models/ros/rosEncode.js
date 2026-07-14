import { decodeMissionRoute } from '../mission/missionEncodeConfig.js';
import { MissionToPsdkV2 } from '../mission/missionEncodePsdk.js';

// Drops keys whose value is undefined, so a param the profile doesn't expose
// (e.g. max_vel / mode_gimbal on the v1 profile) is OMITTED from the message
// instead of sent as `undefined`.
function omitUndefined(obj) {
  return Object.fromEntries(Object.entries(obj).filter(([, v]) => v !== undefined));
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

// ─── Encoder: route crudo → aerialcore_common/ConfigMission (ROS1) ───────────

function MissionToRos(route) {
  const {
    waypoint,
    yaw,
    speed,
    gimbalPitch,
    commandList,
    commandParameter,
    maxVel,
    idleVel,
    yawMode,
    traceMode,
    gimbalPitchMode,
    finishAction,
  } = decodeMissionRoute(route);

  return omitUndefined({
    type: 'waypoint',
    waypoint,
    radius: 0,
    maxVel,
    idleVel,
    yaw: { data: yaw },
    speed: { data: speed },
    gimbalPitch: { data: gimbalPitch },
    yawMode,
    traceMode,
    gimbalPitchMode,
    finishAction,
    commandList: { data: commandList.flat() },
    commandParameter: { data: commandParameter.flat() },
  });
}

// ─── Encoder: route crudo → muav_gcs_interfaces/srv/LoadMission (ROS2) ───────

function MissionToRos2(route) {
  const msg = MissionToRos(route);
  return {
    request: omitUndefined({
      type: 'waypoint',
      waypoint: msg.waypoint,
      radius: msg.radius,
      vel_max: msg.maxVel,
      vel_idle: msg.idleVel,
      yaw: msg.yaw,
      gimbal_pitch: msg.gimbalPitch,
      speed: msg.speed,
      yaw_mode: msg.yawMode,
      trace_mode: msg.traceMode,
      gimbal_pitch_mode: msg.gimbalPitchMode,
      finish_action: msg.finishAction,
      command_list: msg.commandList,
      command_parameter: msg.commandParameter,
    }),
  };
}

// ─── Dispatch: msgType → encoder ─────────────────────────────────────────────

const MISSION_ENCODERS = {
  'aerialcore_common/ConfigMission': MissionToRos,
  'multiuav_interfaces/ConfigMission': MissionToRos,
  'muav_gcs_interfaces/srv/LoadMission': MissionToRos2,
  'psdk_interfaces/srv/InitWaypointV2Setting': MissionToPsdkV2,
};

export function encodeRosSrv({ type, msg, msgType }) {
  if (msg == null) return {};

  if (type === 'configureMission' && MISSION_ENCODERS[msgType]) {
    const mission_encode = MISSION_ENCODERS[msgType](msg);
    return mission_encode;
  }

  if (msgType === 'std_srvs/TriggerRequest') return {};
  if (msgType === 'psdk_interfaces/srv/StartWaypointV2Mission') return {};
  if (msgType === 'psdk_interfaces/srv/StopWaypointV2Mission') return {};
  if (msgType === 'psdk_interfaces/srv/PauseWaypointV2Mission') return {};
  if (msgType === 'psdk_interfaces/srv/ResumeWaypointV2Mission') return {};

  if (msgType === 'px4_msgs/msg/TrajectorySetpoint') {
    return {
      timestamp: msg.timestamp || 0,
      position: msg.position || [0, 0, 0],
      velocity: msg.velocity || [0, 0, 0],
      acceleration: msg.acceleration || [0, 0, 0],
      jerk: msg.jerk || [0, 0, 0],
      yaw: msg.yaw || 0,
      yawspeed: msg.yawspeed || 0,
    };
  }

  if (msgType === 'geometry_msgs/Twist') {
    return { linear: msg.linear || { x: 0, y: 0, z: 0 }, angular: msg.angular || { x: 0, y: 0, z: 0 } };
  }

  // Gimbal (DJI OSDK, ROS1). Wire format de dji_osdk_ros/GimbalAction. El objeto
  // de dominio llega neutro ({pitch, roll, yaw, reset} en grados); acá se traduce.
  if (msgType === 'dji_osdk_ros/GimbalAction') {
    return {
      header: { seq: 0, stamp: { secs: 0, nsecs: 0 }, frame_id: '' },
      is_reset: msg.reset ? true : false,
      payload_index: 0,
      // rotationMode 0 = ángulo relativo al punto de referencia previo, 1 = punto actual.
      rotationMode: 0,
      pitch: msg.pitch || 0.0,
      roll: msg.roll || 0.0,
      yaw: msg.yaw || 0.0,
      time: 0.0,
    };
  }

  // Gimbal (DJI PSDK, ROS2). Wire format de psdk_interfaces/msg/GimbalRotation.
  // Mismo objeto de dominio neutro; PSDK espera RADIANES y rotation_mode
  // (0=incremental, 1=absoluto ground, 2=velocidad). Default 1 (absoluto).
  if (msgType === 'psdk_interfaces/msg/GimbalRotation') {
    return {
      payload_index: msg.payload_index || 1,
      rotation_mode: msg.rotation_mode ?? 1,
      pitch: deg2rad(msg.pitch || 0),
      roll: deg2rad(msg.roll || 0),
      yaw: deg2rad(msg.yaw || 0),
      time: 0.0,
    };
  }
  if (msgType === 'muav_gcs_interfaces/action/DownloadFilesByDateRange') {
    return {
      payload_index: msg.payload_index || 1,
      init_date: assertUtcIsoDate(msg.startDate, 'init_date'),
      finish_date: assertUtcIsoDate(msg.endDate, 'finish_date'),
      file_type: msg.file_type || 'all',
      delete_after_download: msg.delete_after_download || false,
    };
  }
  if (msgType === 'dji_osdk_ros/DownloadMedia') {
    return {
      initDate: assertUtcIsoDate(msg.startDate, 'initDate'),
      FinishDate: assertUtcIsoDate(msg.endDate, 'finishDate'),
      downloadCnt: 0,
    };
  }
  return msg;
}
