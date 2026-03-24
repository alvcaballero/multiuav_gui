import { devicesController } from '../controllers/devices.js';
import { categoryController } from '../controllers/category.js';
import logger from '../common/logger.js';

const ROUTE_DEFAULTS = {
  idle_vel: 1.8,
  max_vel: 10,
  mode_yaw: 0,
  mode_gimbal: 0,
  mode_trace: 0,
  mode_landing: 0,
};

function extractRouteAttributes(routeAttributes) {
  const attrs = {};
  for (const [key, defaultValue] of Object.entries(ROUTE_DEFAULTS)) {
    attrs[key] = routeAttributes.hasOwnProperty(key) ? routeAttributes[key] : defaultValue;
  }
  return attrs;
}

function buildWaypointActions(wpAction, categoryModel) {
  const action_array = Array(10).fill(0);
  const param_array = Array(10).fill(0);
  Object.keys(wpAction).forEach((action_val, index) => {
    const found = Object.values(categoryModel).find((el) => el.name === action_val);
    if (found) {
      action_array[index] = Number(found.id);
      param_array[index] = found.param ? Number(wpAction[action_val]) : 0;
    }
  });
  return { action_array, param_array };
}

function transformWaypoints(waypoints, idle_vel, categoryModel) {
  const wp_command = [];
  const yaw_pos = [];
  const speed_pos = [];
  const gimbal_pos = [];
  const action_matrix = [];
  const param_matrix = [];

  for (const item of Object.values(waypoints)) {
    logger.debug(`wp action: ${JSON.stringify(item.action ?? 'no action')}`);

    const pos = {
      latitude: item.pos[0],
      longitude: item.pos[1],
      altitude: item.pos[2],
    };
    const yaw = item.hasOwnProperty('yaw') ? item.yaw : 0;
    const speed = item.hasOwnProperty('speed') ? item.speed : idle_vel;
    const gimbal = item.hasOwnProperty('gimbal') ? item.gimbal : 0;

    let action_array = Array(10).fill(0);
    let param_array = Array(10).fill(0);
    if (item.hasOwnProperty('action')) {
      ({ action_array, param_array } = buildWaypointActions(item.action, categoryModel));
    }

    wp_command.push(pos);
    yaw_pos.push(yaw);
    speed_pos.push(speed);
    gimbal_pos.push(gimbal);
    action_matrix.push(action_array);
    param_matrix.push(param_array);
  }

  return { wp_command, yaw_pos, speed_pos, gimbal_pos, action_matrix, param_matrix };
}

export async function decodeMissionMsg({ uav_id, route }) {
  const device = await devicesController.getDevice(uav_id);
  logger.debug(`decodeMissionMsg device: ${JSON.stringify(device)}`);

  if (route['uav'] !== device.name) return null;

  const { idle_vel, max_vel, mode_yaw, mode_gimbal, mode_trace, mode_landing } =
    extractRouteAttributes(route.attributes);

  const categoryModel = await categoryController.getActionsParam({ type: device.category });

  const { wp_command, yaw_pos, speed_pos, gimbal_pos, action_matrix, param_matrix } =
    transformWaypoints(route['wp'], idle_vel, categoryModel);

  return {
    type: 'waypoint',
    waypoint: wp_command,
    radius: 0,
    maxVel: max_vel,
    idleVel: idle_vel,
    yaw: yaw_pos,
    speed: speed_pos,
    gimbalPitch: gimbal_pos,
    yawMode: mode_yaw,
    traceMode: mode_trace,
    gimbalPitchMode: mode_gimbal,
    finishAction: mode_landing,
    commandList: action_matrix,
    commandParameter: param_matrix,
  };
}
