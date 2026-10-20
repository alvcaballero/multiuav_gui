// Los encoders reales de cada command (instancia de createCommandRegistry, commandEncoders.js).
//
// Cómo se reconoce cada uno (ver commandEncoders.js):
//   - msgType específico → por msgType: VehicleCommand de PX4, gimbal OSDK/PSDK, los
//     serviceTypes de misión (MISSION_SERVICE_TYPES de mission/missionSymbols.js).
//   - msgType genérico (std_srvs/SetBool, Bool, Trigger…) o sin encoding propio → por la key
//     del endpoint en devices_msg.yaml.
//
// Cada encoder devuelve el CONTENIDO de dominio (qué se manda); la forma que define el msgType
// la arma después models/msgEncode.js, la misma para todo protocolo (commandsModel.sendToDevice).
// Los attributes llegan ya validados y con defaults (command_schema, commandsModel.sendCommand).

import { createCommandRegistry } from './commandEncoders.js';
import { CommandType } from '../../config/commandTypes.js';
import { ServiceKey } from '../../config/deviceMsgCatalog.js';
import { buildVehicleCommand } from './vehicleCommand.js';
import { normalizeMission } from '../mission/taskGraph.js';
import { MISSION_SERVICE_TYPES } from '../mission/missionSymbols.js';

export const commandRegistry = createCommandRegistry();
const { register, registerLocal } = commandRegistry;

const GIMBAL_MSG_TYPES = ['dji_osdk_ros/GimbalAction', 'psdk_interfaces/msg/GimbalRotation'];
const PX4_VEHICLE_COMMAND = 'px4_msgs/msg/VehicleCommand';

// encodeMsg lo vuelve el request vacío ({}). Sobre std_srvs/SetBool o Bool eso es `data: false`,
// y es intencional: los services sin argumentos no miran `data` (CommandMission, que sí lo
// necesita, manda { data: true } con su propio encoder).
const noRequest = () => undefined;
const passAttributes = ({ attributes }) => attributes;

// ─── local ────────────────────────────────────────────────────────────────────
// Sin imports de controllers: category.js carga este módulo para validar al boot. Las
// dependencias llegan en el ctx (commandsModel.sendCommand pasa `positions`).
registerLocal(CommandType.SAVE_HOME, ({ deviceId, positions }) => {
  positions.updatePosition({ deviceId, setHome: true });
  return { state: 'success', msg: 'Home saved' };
});

// ─── flight: PX4 VehicleCommand ───────────────────────────────────────────────
for (const type of [
  CommandType.TAKEOFF,
  CommandType.LAND,
  CommandType.RETURN_HOME,
  CommandType.HOLD,
  CommandType.ARM,
  CommandType.DISARM,
  CommandType.KILL_MOTORS,
  CommandType.MOVE_RELATIVE,
]) {
  register(type, { msgType: PX4_VEHICLE_COMMAND }, ({ attributes, position }) =>
    buildVehicleCommand({ type, attributes, position })
  );
}

// ─── gimbal ───────────────────────────────────────────────────────────────────
// Objeto de dominio NEUTRO: ángulos ABSOLUTOS en grados; msgEncode.js lo traduce por msgType.
// Los dos wire formats llevan siempre los 3 ejes: el eje que no viene se completa con el
// ángulo actual (attributes.gimbal = [pitch, roll, yaw]). Sin telemetría de gimbal la
// posición trae [0, 0, 0].
function gimbalRequest({ attributes = {}, position }, reset = false) {
  const [pitch, roll, yaw] = position?.attributes?.gimbal ?? [0, 0, 0];
  return {
    reset,
    pitch: attributes.pitch ?? pitch,
    roll: attributes.roll ?? roll,
    yaw: attributes.yaw ?? yaw,
  };
}
register(CommandType.GIMBAL, { msgType: GIMBAL_MSG_TYPES }, (ctx) => gimbalRequest(ctx));
register(CommandType.GIMBAL_PITCH, { msgType: GIMBAL_MSG_TYPES }, (ctx) => gimbalRequest(ctx));
register(CommandType.RESET_GIMBAL, { msgType: GIMBAL_MSG_TYPES }, (ctx) => gimbalRequest(ctx, true));

// ─── mission ──────────────────────────────────────────────────────────────────

/**
 * La task de UN device dentro de una misión (cualquier forma que acepte normalizeMission:
 * tasks[], route[] o el route[] pelado que postea la tool MCP load_mission_to_uav).
 * Una misión con varias tasks para el device se rechaza en vez de adivinar cuál volar.
 */
export function missionTaskForDevice(missionData, device) {
  if (missionData == null || (Array.isArray(missionData) && missionData.length === 0)) {
    throw new Error('no mission');
  }
  const mission = normalizeMission(Array.isArray(missionData) ? { route: missionData } : missionData);
  const deviceTasks = mission.tasks.filter((t) => t.device === device.name);
  if (deviceTasks.length === 0) throw new Error(`device ${device.name} has no task in this mission`);
  if (deviceTasks.length > 1) {
    const ids = deviceTasks.map((t) => t.task_id).join(', ');
    throw new Error(`device ${device.name} has several tasks (${ids}); load one task at a time`);
  }
  return { ...deviceTasks[0], uav_type: device.category };
}

register(CommandType.LOAD_MISSION, { msgType: MISSION_SERVICE_TYPES }, ({ attributes, device }) =>
  missionTaskForDevice(attributes, device)
);
register(CommandType.COMMAND_MISSION, { key: ServiceKey.COMMAND_MISSION }, () => ({ data: true }));
register(CommandType.UPLOAD_MISSION, { key: ServiceKey.UPLOAD_MISSION }, passAttributes);
register(CommandType.PAUSE_MISSION, { key: ServiceKey.PAUSE_MISSION }, noRequest);
register(CommandType.RESUME_MISSION, { key: ServiceKey.RESUME_MISSION }, noRequest);
register(CommandType.STOP_MISSION, { key: ServiceKey.STOP_MISSION }, noRequest);

// ─── camera ───────────────────────────────────────────────────────────────────
register(CommandType.SETUP_CAMERA, { key: ServiceKey.SETUP_CAMERA }, passAttributes);
register(CommandType.CAMERA_FILE_LIST, { key: ServiceKey.CAMERA_FILE_LIST }, passAttributes);
register(CommandType.CAMERA_FILE_DOWNLOAD, { key: ServiceKey.CAMERA_FILE_DOWNLOAD }, passAttributes);
register(CommandType.SYNC_FILES, { key: ServiceKey.SYNC_FILES }, noRequest);

// ─── flight: navegación / threat ──────────────────────────────────────────────
register(CommandType.NAVIGATE_TO_POSE, { key: ServiceKey.NAVIGATE_TO_POSE }, passAttributes);
register(CommandType.THREAT_CONFIRMATION, { key: ServiceKey.THREAT_CONFIRMATION }, noRequest);
register(CommandType.THREAT_DEFUSE, { key: ServiceKey.THREAT_DEFUSE }, noRequest);
