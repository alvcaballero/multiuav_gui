import { decodeMissionRoute } from '../MissionDecoder.js';
import { MissionToPsdkV2 } from './psdkEncode.js';

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

  return {
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
  };
}

// ─── Encoder: route crudo → muav_gcs_interfaces/srv/LoadMission (ROS2) ───────

function MissionToRos2(route) {
  const msg = MissionToRos(route);
  return {
    request: {
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
    },
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

  return msg;
}
