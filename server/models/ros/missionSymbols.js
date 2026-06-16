// ─── Mission symbol → firmware number tables (neutral module) ─────────────────
//
// Each family of encoder maps a CANONICAL SYMBOL (the `key` declared in
// mission_schema.yaml) to the number its firmware expects. These tables ARE the
// source of truth for "which option does this family support": the catalog may
// offer 6 yaw modes, but a family that only maps 4 supports only those 4.
//
// Lives in its own module so both the encoders (psdkEncode/MissionDecoder) and
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
    // GO_TO_FIRST_WAYPOINT / CONTINUE_UNTIL_STOP not supported by ConfigMission firmware
  },
});

// serviceType (from devices_msg.yaml services.configureMission) → symbol table.
// This is how a category's encoder family is resolved.
const SERVICE_TO_SYMBOLS = {
  'psdk_interfaces/srv/InitWaypointV2Setting': PSDK_SYMBOLS,
  'aerialcore_common/ConfigMission': CONFIG_SYMBOLS,
  'multiuav_interfaces/ConfigMission': CONFIG_SYMBOLS,
  'muav_gcs_interfaces/srv/LoadMission': CONFIG_SYMBOLS,
};

// Returns the symbol table for a configureMission serviceType, or null when the
// service is absent/unknown (no device selected, or a device without a mission
// service). A null result means "don't filter options" — category.js then shows
// the unfiltered 'default' catalog profile as the planning baseline.
export function symbolsForService(serviceType) {
  return SERVICE_TO_SYMBOLS[serviceType] ?? null;
}

// ─── Capability profile derivation (serviceType + category → profile) ─────────
//
// The profile is DERIVED, not declared, so it can never contradict the encoder.
// The serviceType resolves the majority; a small per-category override list
// handles cases where the same serviceType maps to different capabilities
// (e.g. dji_M300 uses ConfigMission like an M210 but supports the v2 profile).
const SERVICE_TO_PROFILE = {
  'psdk_interfaces/srv/InitWaypointV2Setting': 'psdk',
  'aerialcore_common/ConfigMission': 'v2',
  'multiuav_interfaces/ConfigMission': 'v2',
  'muav_gcs_interfaces/srv/LoadMission': 'v2',
};

// Categories whose profile differs from what their serviceType alone implies.
const CATEGORY_PROFILE_OVERRIDES = {
  dji_M210_noetic_rtk: 'v1',
  dji_M210_noetic: 'v1',
  dji_M210_melodic_rtk: 'v1',
  dji_M210_melodic: 'v1',
  px4_mavros: 'v1',
  px4_mavros_slow: 'v1',
  dji_M600: 'default',
};

// Resolves the catalog profile name for a category. Override by category wins;
// otherwise derive from serviceType; falls back to 'default' (no device/service).
export function profileFor(category, serviceType) {
  return CATEGORY_PROFILE_OVERRIDES[category] ?? SERVICE_TO_PROFILE[serviceType] ?? 'default';
}
