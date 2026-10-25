// ─── Mission symbol → firmware number tables (neutral module) ─────────────────
//
// Each family of encoder maps a CANONICAL SYMBOL (the `key` declared in
// mission_schema.yaml) to the number its firmware expects. These tables ARE the
// source of truth for "which option does this family support": the catalog may
// offer 6 yaw modes, but a family that only maps 4 supports only those 4.
//
// Lives in its own module so both the encoders (missionEncodePsdk/missionEncodeConfig) and
// category.js (which filters the catalog per family) can import it without an
// import cycle.

// DJI WaypointV2 / psdk_interfaces firmware numbers.
export const PSDK_SYMBOLS = Object.freeze({
  mode_yaw: {
    HEADING_MODE_AUTO: 0, // follows direction of flight
    HEADING_MODE_FIXED: 1, // locked to heading at first waypoint
    HEADING_MODE_RC: 2, // controlled by RC
    HEADING_MODE_WAYPOINT_CUSTOM: 3, // adapts to next waypoint
    HEADING_MODE_POI: 4, // always points to POI
    HEADING_MODE_GIMBAL_YAW_FOLLOW: 5, // rotates with gimbal yaw
  },
  mode_trace: {
    FLIGHT_PATH_MODE_CURVE: 0, // curve, fly past
    FLIGHT_PATH_MODE_CURVE_AND_STOP: 1, // curve, stop
    FLIGHT_PATH_MODE_STRAIGHT_AND_STOP: 2, // straight, stop
    FLIGHT_PATH_MODE_COORDINATE_TURN: 3, // smooth, no stop
    FLIGHT_PATH_MODE_FIRST_POINT_STRAIGHT: 4, // go to first waypoint in straight line
    FLIGHT_PATH_MODE_STRAIGHT_OUT: 5, // straight exit — only valid for last waypoint
  },
  mode_landing: {
    MISSION_FINISHED_NO_ACTION: 0, // hover at last waypoint
    MISSION_FINISHED_GO_HOME: 1, // return to home
    MISSION_FINISHED_AUTO_LANDING: 2, // land at last waypoint
    MISSION_FINISHED_GO_TO_FIRST_WAYPOINT: 3, // back to first waypoint and hover
    MISSION_FINISHED_CONTINUE_UNTIL_STOP: 4, // hover without ending mission
  },
  mode_turn: {
    TURN_MODE_CLOCKWISE: 0,
    TURN_MODE_COUNTER_CLOCKWISE: 1,
    TURN_MODE_AUTO: 2,
  },
  actions: {
    ACTION_STAY: 0,
    ACTION_PHOTO: 1,
    ACTION_VIDEO_START: 2,
    ACTION_VIDEO_STOP: 3,
    ACTION_YAW: 4,
    ACTION_GIMBAL: 5,
    ACTION_FOCUS: 6,
    ACTION_ZOOM: 7,
  },
});

// aerialcore_common / multiuav_interfaces / muav_gcs ConfigMission firmware numbers.
//
// ⚠️ PLACEHOLDER VALUES: these mirror the catalog numbers (identity). Replace each
// with the REAL value from the aerialcore_common ConfigMission .srv/.msg once
// available. A symbol ABSENT here means the ConfigMission firmware does NOT support
// that mode — the catalog option is filtered out of the UI for these robots.
export const CONFIG_SYMBOLS = Object.freeze({
  mode_yaw: {
    HEADING_MODE_AUTO: 0,
    HEADING_MODE_FIXED: 1,
    HEADING_MODE_RC: 2,
    HEADING_MODE_WAYPOINT_CUSTOM: 3,
    // POI / GIMBAL_YAW_FOLLOW not supported by ConfigMission firmware
  },
  mode_trace: {
    FLIGHT_PATH_MODE_CURVE: 0,
    FLIGHT_PATH_MODE_CURVE_AND_STOP: 1,
    FLIGHT_PATH_MODE_STRAIGHT_AND_STOP: 2,
    // COORDINATE_TURN not supported by ConfigMission firmware
  },
  mode_landing: {
    MISSION_FINISHED_NO_ACTION: 0,
    MISSION_FINISHED_GO_HOME: 1,
    MISSION_FINISHED_AUTO_LANDING: 2,
    MISSION_FINISHED_VISUAL_LANDING: 5, // supported by ConfigMission firmware
    // GO_TO_FIRST_WAYPOINT / CONTINUE_UNTIL_STOP not supported by ConfigMission firmware
  },
  actions: {
    ACTION_STAY: 0,
    ACTION_PHOTO: 1,
    ACTION_VIDEO_START: 2,
    ACTION_VIDEO_STOP: 3,
    ACTION_YAW: 4,
    ACTION_GIMBAL: 5,
  },
});

// MAVLink mission protocol (PX4 and ArduPilot, missionEncodeMavlink.js). Not firmware numbers:
// each symbol says the encoder implements the option, with mission items of its own. Only what
// was flown in SITL is mapped (MAVLINK_PLAN.md › F4); the rest is filtered out of the UI.
export const MAVLINK_SYMBOLS = Object.freeze({
  mode_yaw: {
    HEADING_MODE_AUTO: 0, // the autopilot faces the next waypoint (its default)
    HEADING_MODE_WAYPOINT_CUSTOM: 3, // wp.yaw at each waypoint
  },
  mode_trace: {
    FLIGHT_PATH_MODE_STRAIGHT_AND_STOP: 2,
  },
  mode_landing: {
    MISSION_FINISHED_NO_ACTION: 0, // hover at the last waypoint
    MISSION_FINISHED_GO_HOME: 1, // NAV_RETURN_TO_LAUNCH
    MISSION_FINISHED_AUTO_LANDING: 2, // NAV_LAND at the last waypoint
  },
  actions: {
    ACTION_STAY: 0,
    ACTION_PHOTO: 1,
    ACTION_VIDEO_START: 2,
    ACTION_VIDEO_STOP: 3,
    ACTION_YAW: 4,
    ACTION_GIMBAL: 5,
    ACTION_VTOL_TRANSITION: 8, // DO_VTOL_TRANSITION, only on a VTOL
  },
  vtol_transition: {
    VTOL_TRANSITION_FW: 0,
    VTOL_TRANSITION_MC: 1,
  },
});

// The configure_mission messageType of the mavlink categories. One per dialect, like the
// commands: what a mission item means differs between PX4 and ArduPilot (home at seq 0).
export const MAVLINK_MISSION = Object.freeze({
  PX4: 'mavlink/PX4_MISSION',
  ARDUPILOT: 'mavlink/ARDUPILOT_MISSION',
});

// serviceType (the messageType of devices_msg.yaml services.configure_mission) → symbol table.
// This is how a category's encoder family is resolved.
const SERVICE_TO_SYMBOLS = {
  'psdk_interfaces/srv/InitWaypointV2Setting': PSDK_SYMBOLS,
  'aerialcore_common/ConfigMission': CONFIG_SYMBOLS,
  'multiuav_interfaces/ConfigMission': CONFIG_SYMBOLS,
  'muav_gcs_interfaces/srv/LoadMission': CONFIG_SYMBOLS,
  [MAVLINK_MISSION.PX4]: MAVLINK_SYMBOLS,
  [MAVLINK_MISSION.ARDUPILOT]: MAVLINK_SYMBOLS,
};

// The serviceTypes a configure_mission endpoint can declare: msgEncode.js encodes a mission for
// each, and commandRegistry.js recognizes LoadMission by them. Lives here (a module with no
// imports) so neither of them imports the other, which would close a cycle through category.js.
export const MISSION_SERVICE_TYPES = Object.freeze(Object.keys(SERVICE_TO_SYMBOLS));

// Returns the symbol table for a configure_mission serviceType, or null when the
// service is absent/unknown (no device selected, or a device without a mission
// service). A null result means "don't filter options" — category.js then shows
// the unfiltered 'default' catalog profile as the planning baseline.
export function symbolsForService(serviceType) {
  return SERVICE_TO_SYMBOLS[serviceType] ?? null;
}

// ─── Capability profile derivation (serviceType + category → profile) ─────────
//
// The profile is DERIVED, not declared, so it can never contradict the encoder.
// The serviceType resolves the majority; a category whose firmware supports less than
// its serviceType implies (dji_M210 uses ConfigMission like an M300 but only v1) sets
// `mission:` in its interface file (categoryModel's resolveMissionProfile).
const SERVICE_TO_PROFILE = {
  'psdk_interfaces/srv/InitWaypointV2Setting': 'psdk',
  'aerialcore_common/ConfigMission': 'v2',
  'multiuav_interfaces/ConfigMission': 'v2',
  'muav_gcs_interfaces/srv/LoadMission': 'v2',
  [MAVLINK_MISSION.PX4]: 'mavlink',
  [MAVLINK_MISSION.ARDUPILOT]: 'mavlink',
};

// The catalog profile name a configure_mission serviceType implies; 'default' when
// there is none (no mission service).
export function profileFor(serviceType) {
  return SERVICE_TO_PROFILE[serviceType] ?? 'default';
}
