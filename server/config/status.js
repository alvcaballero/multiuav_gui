// Definimos el mapa de estados
const navigationStates = {
  0: "Manual mode",
  1: "Altitude mode",
  2: "Position mode",
  3: "Mission mode",
  4: "Loiter mode",
  5: "Auto return to launch mode",
  6: "Position slow",
  7: "Free5",
  8: "Free4",
  9: "Free3",
  10: "Acro mode",
  11: "Free2",
  12: "Descend mode",
  13: "Termination mode",
  14: "Offboard mode",
  15: "Stabilized mode",
  16: "Free1",
  17: "Auto Takeoff",
  18: "Auto Land",
  19: "Auto Follow",
  20: "Precision Land",
  21: "Orbit mode",
  22: "VTOL Takeoff",
  23: "External1",
  24: "External2",
  25: "External3",
  26: "External4",
  27: "External5",
  28: "External6",
  29: "External7",
  30: "External8",
  31: "Max"
};

const armingStates = {
  1: "Disarmed",
  2: "Armed",
}

const vehicleCommands = {
  0: "CUSTOM_0 (Test command)",
  1: "CUSTOM_1 (Test command)",
  2: "CUSTOM_2 (Test command)",
  16: "NAV_WAYPOINT (Navigate to MISSION)",
  17: "NAV_LOITER_UNLIM (Loiter unlimited)",
  18: "NAV_LOITER_TURNS (Loiter for X turns)",
  19: "NAV_LOITER_TIME (Loiter for time)",
  20: "NAV_RETURN_TO_LAUNCH (RTL)",
  21: "NAV_LAND (Land at location)",
  22: "NAV_TAKEOFF (Takeoff)",
  23: "NAV_PRECLAND (Precision Land)",
  34: "DO_ORBIT",
  35: "DO_FIGUREEIGHT",
  80: "NAV_ROI (Region of Interest)",
  81: "NAV_PATHPLANNING",
  84: "NAV_VTOL_TAKEOFF",
  85: "NAV_VTOL_LAND",
  176: "DO_SET_MODE",
  177: "DO_JUMP",
  178: "DO_CHANGE_SPEED",
  179: "DO_SET_HOME",
  180: "DO_SET_PARAMETER",
  181: "DO_SET_RELAY",
  182: "DO_REPEAT_RELAY",
  184: "DO_REPEAT_SERVO",
  185: "DO_FLIGHTTERMINATION",
  186: "DO_CHANGE_ALTITUDE",
  187: "DO_SET_ACTUATOR",
  189: "DO_LAND_START",
  191: "DO_GO_AROUND",
  192: "DO_REPOSITION",
  193: "DO_PAUSE_CONTINUE",
  200: "DO_CONTROL_VIDEO",
  201: "DO_SET_ROI",
  203: "DO_DIGICAM_CONTROL",
  205: "DO_MOUNT_CONTROL",
  206: "DO_SET_CAM_TRIGG_DIST",
  207: "DO_FENCE_ENABLE",
  208: "DO_PARACHUTE",
  209: "DO_MOTOR_TEST",
  210: "DO_INVERTED_FLIGHT",
  211: "DO_GRIPPER",
  214: "DO_SET_CAM_TRIGG_INTERVAL",
  220: "DO_MOUNT_CONTROL_QUAT",
  240: "DO_LAST",
  241: "PREFLIGHT_CALIBRATION",
  242: "PREFLIGHT_SET_SENSOR_OFFSETS",
  243: "PREFLIGHT_UAVCAN",
  245: "PREFLIGHT_STORAGE",
  246: "PREFLIGHT_REBOOT_SHUTDOWN",
  260: "OBLIQUE_SURVEY",
  262: "DO_SET_STANDARD_MODE",
  283: "GIMBAL_DEVICE_INFORMATION",
  300: "MISSION_START",
  310: "ACTUATOR_TEST",
  311: "CONFIGURE_ACTUATOR",
  400: "COMPONENT_ARM_DISARM",
  401: "RUN_PREARM_CHECKS",
  420: "INJECT_FAILURE",
  500: "START_RX_PAIR",
  512: "REQUEST_MESSAGE",
  521: "REQUEST_CAMERA_INFORMATION",
  530: "SET_CAMERA_MODE",
  531: "SET_CAMERA_ZOOM",
  532: "SET_CAMERA_FOCUS",
  1000: "DO_GIMBAL_MANAGER_PITCHYAW",
  1001: "DO_GIMBAL_MANAGER_CONFIGURE",
  2000: "IMAGE_START_CAPTURE",
  2003: "DO_TRIGGER_CONTROL",
  2500: "VIDEO_START_CAPTURE",
  2501: "VIDEO_STOP_CAPTURE",
  2510: "LOGGING_START",
  2511: "LOGGING_STOP",
  2600: "CONTROL_HIGH_LATENCY",
  3000: "DO_VTOL_TRANSITION",
  3001: "ARM_AUTHORIZATION_REQUEST",
  30001: "PAYLOAD_PREPARE_DEPLOY",
  30002: "PAYLOAD_CONTROL_DEPLOY",
  42006: "FIXED_MAG_CAL_YAW",
  42600: "DO_WINCH",
  43003: "EXTERNAL_POSITION_ESTIMATE",
  43004: "EXTERNAL_WIND_ESTIMATE"
};

const vehicleCmdResults = {
  0: "ACCEPTED and EXECUTED",
  1: "TEMPORARILY REJECTED",
  2: "DENIED (permanently)",
  3: "UNSUPPORTED / UNKNOWN",
  4: "FAILED (executed but failed)",
  5: "IN PROGRESS",
  6: "CANCELLED"
};



// Función para obtener el estado
export function getDroneStatus(code) {
  return navigationStates[code] || "Unknown status";
}
export function getArmingStatus(code) {
  return armingStates[code] || "Unknown status";
}

// Función para obtener el resultado por código
export function getVehicleCmdResult(code) {
  return vehicleCmdResults[code] || "Unknown result";
}

// Función para obtener comando por ID
export function getVehicleCommand(cmd) {
  return vehicleCommands[cmd] || "Unknown command";
}
