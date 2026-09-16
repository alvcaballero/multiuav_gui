// Command catalog — SSOT de los COMANDOS que el usuario puede disparar.
//
// El objeto de dominio central es el COMMAND (una acción de usuario), NO la key
// del devices_msg. Están relacionados pero no son lo mismo:
//
//   - devices_msg key (serviceKey)  — una CAPACIDAD ROS que la categoría del device
//     declara en sus bloques services:/actions:. Es config del dispositivo. La
//     enumera `ServiceKey`. Está fijada por el lado ROS (.srv/.action), no se renombra.
//
//   - command                   — una ACCIÓN que el usuario dispara desde la UI.
//     Tiene un `type` (lo que la UI muestra y sendCommand compara) y su propia
//     forma de despacharse. Un command PUEDE requerir una serviceKey para estar
//     disponible (`requires`), pero no siempre: saveHome/custom no dependen de
//     ninguna. Y una misma serviceKey puede habilitar VARIOS commands (Gimbal → 3).
//
// La relación es command → requires → serviceKey (un requisito), no serviceKey →
// commands. Por eso el catálogo se indexa por command type; `requires` apunta a
// la serviceKey.
//
// Las serviceKeys (keys del devices_msg) NO se definen acá: son config del device
// y viven en deviceMsgCatalog.js. Este módulo las IMPORTA como `ServiceKey` y las
// referencia desde `requires`.
//
// Consumidores:
//   - getCommandTypes(device): filtra los commands cuyo `requires` está en las
//     capacidades del device (o es null → siempre disponible).
//   - sendCommand: despacha según `dispatch`/`payload`, usando `rosService`.

import { ServiceKey } from './deviceMsgCatalog.js';

// ─── Enums (SSOT de literales de commands) ────────────────────────────────────

// command types — el `type` que la UI muestra y que sendCommand compara.
// Distinto de ServiceKey: incluye commands sin serviceKey (saveHome/custom) y
// varios commands que comparten una misma serviceKey (los de gimbal).
export const CommandType = Object.freeze({
  SAVE_HOME: 'saveHome',
  CUSTOM: 'custom',
  LOAD_MISSION: 'loadMission',
  COMMAND_MISSION: 'commandMission',
  SINCRONISE_FILES: 'SincroniseFiles',
  RESUME_MISSION: 'ResumeMission',
  STOP_MISSION: 'StopMission',
  PAUSE_MISSION: 'Pausemission',
  SETUP_CAMERA: 'setupcamera',
  GIMBAL: 'Gimbal',
  GIMBAL_PITCH: 'GimbalPitch',
  RESET_GIMBAL: 'ResetGimbal',
  CAMERA_FILE_DOWNLOAD: 'CameraFileDownload',
  CAMERA_FILE_LIST: 'CameraFileList',
  UPLOAD_MISSION: 'uploadMission',
  NAVIGATE_TO_POSE: 'navigateToPose',
  THREAT_CONFIRMATION: 'threat_confirmation',
  THREAT_DEFUSE: 'threat_defuse',
});

// Cómo sendCommand ejecuta el command.
export const Dispatch = Object.freeze({
  LOCAL: 'local', // maneja el server sin ROS (ej. saveHome → positionsController)
  FLEET: 'fleet', // handler propio de flota (loadMission/commandMission)
  SERVICE: 'service', // ROS service vía standarCommand(rosService)
  GIMBAL: 'gimbal', // ruta especial GimbalUAV
});

// Cómo se arma el request del ROS service.
export const Payload = Object.freeze({
  NONE: 'none', // sin request
  ATTRIBUTES: 'attributes', // los attributes crudos
  RESET: 'reset', // { reset: true } (ResetGimbal)
});

// ─── Catálogo (indexado por command type) ─────────────────────────────────────

// command type → definición.
//   requires:   serviceKey que el device debe tener para exponer este command
//               (null = siempre disponible, no depende del devices_msg).
//   dispatch:   cómo lo ejecuta sendCommand (Dispatch.*).
//   rosService: string que se pasa a standarCommand cuando dispatch es SERVICE
//               (normalmente == requires; explícito para no re-derivarlo).
//   payload:    cómo se arma el request (Payload.*).
export const COMMAND_CATALOG = {
  [CommandType.SAVE_HOME]: { requires: null, dispatch: Dispatch.LOCAL },
  [CommandType.CUSTOM]: {
    requires: null,
    dispatch: Dispatch.SERVICE,
    rosService: undefined,
    payload: Payload.ATTRIBUTES,
  },
  [CommandType.LOAD_MISSION]: { requires: ServiceKey.CONFIGURE_MISSION, dispatch: Dispatch.FLEET },
  [CommandType.COMMAND_MISSION]: { requires: ServiceKey.COMMAND_MISSION, dispatch: Dispatch.FLEET },
  [CommandType.SINCRONISE_FILES]: {
    requires: ServiceKey.SINCRONIZE,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.SINCRONIZE,
    payload: Payload.NONE,
  },
  [CommandType.RESUME_MISSION]: {
    requires: ServiceKey.RESUME_MISSION,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.RESUME_MISSION,
    payload: Payload.NONE,
  },
  [CommandType.STOP_MISSION]: {
    requires: ServiceKey.STOP_MISSION,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.STOP_MISSION,
    payload: Payload.NONE,
  },
  [CommandType.PAUSE_MISSION]: {
    requires: ServiceKey.PAUSE_MISSION,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.PAUSE_MISSION,
    payload: Payload.NONE,
  },
  [CommandType.SETUP_CAMERA]: {
    requires: ServiceKey.SETUP_CAMERA,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.SETUP_CAMERA,
    payload: Payload.ATTRIBUTES,
  },
  [CommandType.GIMBAL]: { requires: ServiceKey.GIMBAL, dispatch: Dispatch.GIMBAL, payload: Payload.ATTRIBUTES },
  [CommandType.GIMBAL_PITCH]: { requires: ServiceKey.GIMBAL, dispatch: Dispatch.GIMBAL, payload: Payload.ATTRIBUTES },
  [CommandType.RESET_GIMBAL]: { requires: ServiceKey.GIMBAL, dispatch: Dispatch.GIMBAL, payload: Payload.RESET },
  [CommandType.CAMERA_FILE_DOWNLOAD]: {
    requires: ServiceKey.CAMERA_FILE_DOWNLOAD,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.CAMERA_FILE_DOWNLOAD,
    payload: Payload.ATTRIBUTES,
  },
  [CommandType.CAMERA_FILE_LIST]: {
    requires: ServiceKey.CAMERA_FILE_LIST,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.CAMERA_FILE_LIST,
    payload: Payload.ATTRIBUTES,
  },
  [CommandType.UPLOAD_MISSION]: {
    requires: ServiceKey.UPLOAD_MISSION,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.UPLOAD_MISSION,
    payload: Payload.ATTRIBUTES,
  },
  [CommandType.NAVIGATE_TO_POSE]: {
    requires: ServiceKey.NAVIGATE_TO_POSE,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.NAVIGATE_TO_POSE,
    payload: Payload.ATTRIBUTES,
  },
  [CommandType.THREAT_CONFIRMATION]: {
    requires: ServiceKey.THREAT_CONFIRMATION,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.THREAT_CONFIRMATION,
    payload: Payload.NONE,
  },
  [CommandType.THREAT_DEFUSE]: {
    requires: ServiceKey.THREAT_DEFUSE,
    dispatch: Dispatch.SERVICE,
    rosService: ServiceKey.THREAT_DEFUSE,
    payload: Payload.NONE,
  },
};

// ─── Índices derivados y helpers ──────────────────────────────────────────────

// Commands siempre disponibles (requires == null). getCommandTypes los incluye
// para cualquier device.
export const DEFAULT_COMMAND_TYPES = Object.freeze(
  Object.entries(COMMAND_CATALOG)
    .filter(([, def]) => def.requires == null)
    .map(([type]) => type)
);

// serviceKey → command types que la habilitan. Lo usa getCommandTypes: dado el
// set de capacidades del device, junta los commands cuyo `requires` matchea.
export const SERVICE_KEY_TO_TYPES = Object.freeze(
  Object.entries(COMMAND_CATALOG).reduce((acc, [type, def]) => {
    if (def.requires == null) return acc;
    (acc[def.requires] ??= []).push(type);
    return acc;
  }, {})
);

// command types conocidos (todas las keys del catálogo).
export const KNOWN_COMMAND_TYPES = new Set(Object.keys(COMMAND_CATALOG));

// Command types que expone una serviceKey del device, o [] si ninguna la referencia.
export function typesForServiceKey(serviceKey) {
  return SERVICE_KEY_TO_TYPES[serviceKey] ?? [];
}

// Definición de despacho de un command type (para sendCommand), o undefined.
export function commandDef(type) {
  return COMMAND_CATALOG[type];
}
