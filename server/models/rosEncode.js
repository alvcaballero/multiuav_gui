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

export function encodeRosSrv({ type, msg, msgType }) {
  if (type == 'configureMission') {
    return MissionToRos(msg);
  }
  if (msgType == 'std_srvs/TriggerRequest') {
    return {};
  }

  return msg;
}
