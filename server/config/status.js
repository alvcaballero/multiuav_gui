// --- Device status ---
export const DEVICE_STATUS = Object.freeze({
  ONLINE: 'online',
  OFFLINE: 'offline',
});

// --- Mission / Task status ---
export const MISSION_STATUS = Object.freeze({
  INIT: 'init',
  PLANNING: 'planning',
  RUNNING: 'running',
  COMPLETED: 'finish', // UAV finished but files not yet downloaded
  COMPLETED_WITH_ERRORS: 'finish_errors', // active routes finished, but some route errored (never flew)
  END: 'done', // UAV finished and files downloaded
  CANCELLED: 'cancelled',
  ERROR: 'error',
});

export const TASK_STATUS = Object.freeze({
  INIT: 'init', // created, waiting for its dependencies
  LOADED: 'loaded',
  COMMANDED: 'commanded',
  RUNNING: 'running',
  COMPLETED: 'complete', // device reached the last waypoint (satisfies dependents)
  END: 'end', // UAV finished and files downloaded
  CANCELLED: 'cancelled',
  ERROR: 'error',
  SKIPPED: 'skipped', // never ran: an upstream dependency failed or was cancelled
});

// Missions that are still "alive" (not in a terminal state). A re-sent external
// task that maps to one of these is a duplicate; anything else (cancelled/error/
// finished) is terminal and a re-send is treated as a fresh attempt.
export const MISSION_ALIVE_STATUS = Object.freeze([
  MISSION_STATUS.INIT,
  MISSION_STATUS.PLANNING,
  MISSION_STATUS.RUNNING,
]);

// A task holding its device: loaded on it or flying. The task graph guarantees at
// most one per device, which is what lets a device-only ROS signal resolve its task.
export const TASK_ACTIVE_STATUS = Object.freeze([TASK_STATUS.LOADED, TASK_STATUS.COMMANDED, TASK_STATUS.RUNNING]);

// A task that will never change again on its own; the mission is over once every
// task is in one of these.
export const TASK_TERMINAL_STATUS = Object.freeze([
  TASK_STATUS.COMPLETED,
  TASK_STATUS.END,
  TASK_STATUS.CANCELLED,
  TASK_STATUS.ERROR,
  TASK_STATUS.SKIPPED,
]);

// Terminal tasks that never flew to completion: they make a finished mission "with errors".
export const TASK_FAILED_STATUS = Object.freeze([TASK_STATUS.CANCELLED, TASK_STATUS.ERROR, TASK_STATUS.SKIPPED]);

// --- Canonical vehicle state ---
// The GCS vocabulary: every decoder translates its message's own codes into these, so no
// consumer ever compares against a firmware's label. Three independent axes; UNKNOWN means
// "the source says nothing", OTHER means "it says something the GCS does not tell apart".
// The firmware's raw code travels next to the canonical value for debugging.
export const ARM_STATE = Object.freeze({
  ARMED: 'ARMED',
  DISARMED: 'DISARMED',
  UNKNOWN: 'UNKNOWN',
});

// Where the vehicle is, regardless of who flies it.
export const FLIGHT_PHASE = Object.freeze({
  ON_GROUND: 'ON_GROUND',
  TAKEOFF: 'TAKEOFF',
  IN_AIR: 'IN_AIR',
  LANDING: 'LANDING',
  UNKNOWN: 'UNKNOWN',
});

// How the vehicle is being flown. Deliberately coarser than any firmware's mode list.
export const FLIGHT_MODE = Object.freeze({
  MANUAL: 'MANUAL', // pilot commands the attitude/rates directly
  ASSISTED: 'ASSISTED', // pilot flies, the autopilot stabilises (altitude/position hold)
  HOLD: 'HOLD',
  MISSION: 'MISSION',
  TAKEOFF: 'TAKEOFF',
  LAND: 'LAND',
  RTL: 'RTL',
  AUTO_OTHER: 'AUTO_OTHER', // autonomous behaviour the GCS does not tell apart (orbit, follow…)
  OFFBOARD: 'OFFBOARD', // an external computer (companion/GCS) sends the setpoints
  EMERGENCY: 'EMERGENCY', // flight termination
  OTHER: 'OTHER',
  UNKNOWN: 'UNKNOWN',
});

// Who is in control, derived from the mode: one value stored, nothing to keep consistent.
export const CONTROL_LEVEL = Object.freeze({
  MANUAL: 'MANUAL',
  ASSISTED: 'ASSISTED',
  AUTO: 'AUTO', // the autopilot's own logic
  EXTERNAL: 'EXTERNAL', // an external computer
  UNKNOWN: 'UNKNOWN',
});

const MODE_CONTROL = Object.freeze({
  [FLIGHT_MODE.MANUAL]: CONTROL_LEVEL.MANUAL,
  [FLIGHT_MODE.ASSISTED]: CONTROL_LEVEL.ASSISTED,
  [FLIGHT_MODE.HOLD]: CONTROL_LEVEL.AUTO,
  [FLIGHT_MODE.MISSION]: CONTROL_LEVEL.AUTO,
  [FLIGHT_MODE.TAKEOFF]: CONTROL_LEVEL.AUTO,
  [FLIGHT_MODE.LAND]: CONTROL_LEVEL.AUTO,
  [FLIGHT_MODE.RTL]: CONTROL_LEVEL.AUTO,
  [FLIGHT_MODE.AUTO_OTHER]: CONTROL_LEVEL.AUTO,
  [FLIGHT_MODE.OFFBOARD]: CONTROL_LEVEL.EXTERNAL,
  [FLIGHT_MODE.EMERGENCY]: CONTROL_LEVEL.UNKNOWN, // terminated: nobody is flying it
  [FLIGHT_MODE.OTHER]: CONTROL_LEVEL.UNKNOWN,
  [FLIGHT_MODE.UNKNOWN]: CONTROL_LEVEL.UNKNOWN,
});

export const controlLevelOf = (mode) => MODE_CONTROL[mode] ?? CONTROL_LEVEL.UNKNOWN;

// What the vehicle answered to a command (an ack), in the GCS vocabulary like the axes above.
export const COMMAND_RESULT = Object.freeze({
  ACCEPTED: 'ACCEPTED', // accepted and executed
  REJECTED: 'REJECTED', // temporarily: retrying later may work
  DENIED: 'DENIED', // permanently
  UNSUPPORTED: 'UNSUPPORTED',
  FAILED: 'FAILED', // accepted, executed, and it failed
  IN_PROGRESS: 'IN_PROGRESS', // not an answer yet: more acks follow
  CANCELLED: 'CANCELLED',
  UNKNOWN: 'UNKNOWN',
});

/** True when something other than the pilot's sticks is flying: the autopilot or an external computer. */
export const isAutonomous = (mode) => {
  const level = controlLevelOf(mode);
  return level === CONTROL_LEVEL.AUTO || level === CONTROL_LEVEL.EXTERNAL;
};

// --- PX4 command maps ---
const vehicleCommands = {
  0: 'CUSTOM_0 (Test command)',
  1: 'CUSTOM_1 (Test command)',
  2: 'CUSTOM_2 (Test command)',
  16: 'NAV_WAYPOINT (Navigate to MISSION)',
  17: 'NAV_LOITER_UNLIM (Loiter unlimited)',
  18: 'NAV_LOITER_TURNS (Loiter for X turns)',
  19: 'NAV_LOITER_TIME (Loiter for time)',
  20: 'NAV_RETURN_TO_LAUNCH (RTL)',
  21: 'NAV_LAND (Land at location)',
  22: 'NAV_TAKEOFF (Takeoff)',
  23: 'NAV_PRECLAND (Precision Land)',
  34: 'DO_ORBIT',
  35: 'DO_FIGUREEIGHT',
  80: 'NAV_ROI (Region of Interest)',
  81: 'NAV_PATHPLANNING',
  84: 'NAV_VTOL_TAKEOFF',
  85: 'NAV_VTOL_LAND',
  176: 'DO_SET_MODE',
  177: 'DO_JUMP',
  178: 'DO_CHANGE_SPEED',
  179: 'DO_SET_HOME',
  180: 'DO_SET_PARAMETER',
  181: 'DO_SET_RELAY',
  182: 'DO_REPEAT_RELAY',
  184: 'DO_REPEAT_SERVO',
  185: 'DO_FLIGHTTERMINATION',
  186: 'DO_CHANGE_ALTITUDE',
  187: 'DO_SET_ACTUATOR',
  189: 'DO_LAND_START',
  191: 'DO_GO_AROUND',
  192: 'DO_REPOSITION',
  193: 'DO_PAUSE_CONTINUE',
  200: 'DO_CONTROL_VIDEO',
  201: 'DO_SET_ROI',
  203: 'DO_DIGICAM_CONTROL',
  205: 'DO_MOUNT_CONTROL',
  206: 'DO_SET_CAM_TRIGG_DIST',
  207: 'DO_FENCE_ENABLE',
  208: 'DO_PARACHUTE',
  209: 'DO_MOTOR_TEST',
  210: 'DO_INVERTED_FLIGHT',
  211: 'DO_GRIPPER',
  214: 'DO_SET_CAM_TRIGG_INTERVAL',
  220: 'DO_MOUNT_CONTROL_QUAT',
  240: 'DO_LAST',
  241: 'PREFLIGHT_CALIBRATION',
  242: 'PREFLIGHT_SET_SENSOR_OFFSETS',
  243: 'PREFLIGHT_UAVCAN',
  245: 'PREFLIGHT_STORAGE',
  246: 'PREFLIGHT_REBOOT_SHUTDOWN',
  260: 'OBLIQUE_SURVEY',
  262: 'DO_SET_STANDARD_MODE',
  283: 'GIMBAL_DEVICE_INFORMATION',
  300: 'MISSION_START',
  310: 'ACTUATOR_TEST',
  311: 'CONFIGURE_ACTUATOR',
  400: 'COMPONENT_ARM_DISARM',
  401: 'RUN_PREARM_CHECKS',
  420: 'INJECT_FAILURE',
  500: 'START_RX_PAIR',
  512: 'REQUEST_MESSAGE',
  521: 'REQUEST_CAMERA_INFORMATION',
  530: 'SET_CAMERA_MODE',
  531: 'SET_CAMERA_ZOOM',
  532: 'SET_CAMERA_FOCUS',
  1000: 'DO_GIMBAL_MANAGER_PITCHYAW',
  1001: 'DO_GIMBAL_MANAGER_CONFIGURE',
  2000: 'IMAGE_START_CAPTURE',
  2003: 'DO_TRIGGER_CONTROL',
  2500: 'VIDEO_START_CAPTURE',
  2501: 'VIDEO_STOP_CAPTURE',
  2510: 'LOGGING_START',
  2511: 'LOGGING_STOP',
  2600: 'CONTROL_HIGH_LATENCY',
  3000: 'DO_VTOL_TRANSITION',
  3001: 'ARM_AUTHORIZATION_REQUEST',
  30001: 'PAYLOAD_PREPARE_DEPLOY',
  30002: 'PAYLOAD_CONTROL_DEPLOY',
  42006: 'FIXED_MAG_CAL_YAW',
  42600: 'DO_WINCH',
  43003: 'EXTERNAL_POSITION_ESTIMATE',
  43004: 'EXTERNAL_WIND_ESTIMATE',
};

// Función para obtener comando por ID
export function getVehicleCommand(cmd) {
  return vehicleCommands[cmd] || 'Unknown command';
}
