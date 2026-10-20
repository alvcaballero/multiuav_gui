// Command types — SSOT de los literales de los COMMANDS que el usuario (o el LLM) puede
// disparar: el `type` de POST /api/commands/send y de GET /api/commands/types.
//
// Un command es una ACCIÓN de dominio (Takeoff), no un mensaje: cada plataforma la implementa
// con el suyo. Lo demás de un command vive en otro lado, cada cosa una sola vez:
//   - qué significa (params, unidades, rangos, label, confirm) → config/devices/command_schema.yaml
//   - qué endpoint de un device lo implementa y cómo se arma su mensaje → models/commands/commandRegistry.js
//     (registry de models/commands/commandEncoders.js; la disponibilidad se deriva de los endpoints que
//     la categoría declara en devices_msg.yaml)
//
// Todos en PascalCase. No confundir con las keys de devices_msg.yaml (ServiceKey/PublisherKey,
// deviceMsgCatalog.js): CommandMission es la acción, `command_mission` es el endpoint que la
// implementa en algunos drones (las keys van en snake_case).
//
// El cliente tiene un espejo en client/src/common/util/commandTypes.js
// (test/test-command-types-mirror.js verifica que coinciden).
export const CommandType = Object.freeze({
  SAVE_HOME: 'SaveHome',
  LOAD_MISSION: 'LoadMission',
  COMMAND_MISSION: 'CommandMission',
  SYNC_FILES: 'SyncFiles',
  RESUME_MISSION: 'ResumeMission',
  STOP_MISSION: 'StopMission',
  PAUSE_MISSION: 'PauseMission',
  SETUP_CAMERA: 'SetupCamera',
  GIMBAL: 'Gimbal',
  GIMBAL_PITCH: 'GimbalPitch',
  RESET_GIMBAL: 'ResetGimbal',
  CAMERA_FILE_DOWNLOAD: 'CameraFileDownload',
  CAMERA_FILE_LIST: 'CameraFileList',
  UPLOAD_MISSION: 'UploadMission',
  NAVIGATE_TO_POSE: 'NavigateToPose',
  THREAT_CONFIRMATION: 'ThreatConfirmation',
  THREAT_DEFUSE: 'ThreatDefuse',
  TAKEOFF: 'Takeoff',
  LAND: 'Land',
  RETURN_HOME: 'ReturnHome',
  HOLD: 'Hold',
  ARM: 'Arm',
  DISARM: 'Disarm',
  KILL_MOTORS: 'KillMotors',
  MOVE_RELATIVE: 'MoveRelative',
});
