// Command types the GCS understands: the `type` of POST /api/commands/send and of
// GET /api/commands/types. Mirror of `CommandType` in server/config/commandCatalog.js;
// server/test/test-command-types-mirror.js fails if they drift apart.
// Which ones a device supports comes from GET /api/commands/types, never from here.
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
