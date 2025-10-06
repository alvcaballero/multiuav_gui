import { timeStamp } from 'console';
import { request } from 'http';
import ROSLIB from 'roslib';

function MissionToRos({
  yawMode = 0,
  gimbalPitchMode = 0,
  traceMode = 0,
  idleVel = 1.8,
  maxVel = 10,
  finishAction = 0,
  waypoint = [],
  yaw = [],
  speed = [],
  gimbalPitch = [],
  commandList = [],
  commandParameter = [],
}) {
  let wp_command_msg = waypoint.map((pos) => {
    return new ROSLIB.Message(pos);
  });

  let yaw_pos_msg = new ROSLIB.Message({ data: yaw });
  let speed_pos_msg = new ROSLIB.Message({ data: speed });
  let gimbal_pos_msg = new ROSLIB.Message({ data: gimbalPitch });
  let action_matrix_msg = new ROSLIB.Message({
    data: commandList.flat(),
  });
  let param_matrix_msg = new ROSLIB.Message({
    data: commandParameter.flat(),
  });
  return {
    type: 'waypoint',
    waypoint: wp_command_msg,
    radius: 0,
    maxVel: maxVel,
    idleVel: idleVel,
    yaw: yaw_pos_msg,
    speed: speed_pos_msg,
    gimbalPitch: gimbal_pos_msg,
    yawMode: yawMode,
    traceMode: traceMode,
    gimbalPitchMode: gimbalPitchMode,
    finishAction: finishAction,
    commandList: action_matrix_msg,
    commandParameter: param_matrix_msg,
  };
}
function MissionToRos2(param) {
  let msg = MissionToRos(param);
  return {request : new ROSLIB.Message(msg)}
}

export function encodeRosSrv({ type, msg, msgType }) {
  if (type == 'configureMission' && msgType == 'aerialcore_common/ConfigMission') {
    return MissionToRos(msg);
  }
  if (type == 'configureMission' && msgType == 'muav_gcs_interfaces/LoadMission') {
    return MissionToRos2(msg);
  }
  if (msgType == 'std_srvs/TriggerRequest') {
    return {};
  }
  if (msgType == 'px4_msgs/msg/TrajectorySetpoint') {
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
  if (msgType == 'geometry_msgs/Twist') {
    return { linear: msg.linear || { x: 0, y: 0, z: 0 }, angular: msg.angular || { x: 0, y: 0, z: 0 } };
  }

  return msg;
}
