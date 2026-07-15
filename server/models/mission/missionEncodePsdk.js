import { categoryModel } from '../category.js';
import { PSDK_SYMBOLS } from './missionSymbols.js';

// ─── PSDK symbol → firmware number translation ────────────────────────────────
//
// PSDK_SYMBOLS (in missionSymbols.js) maps each CANONICAL SYMBOL (the `key` in
// mission_schema.yaml) to the DJI WaypointV2 firmware number. The mission wire
// carries a number; the catalog turns it into a symbol; toPsdkValue() turns the
// symbol into the firmware number.

// symbol → PSDK firmware number. Fails loud if the catalog offers a symbol this
// family doesn't map (guarded by a contract test so it can't happen silently).
export function toPsdkValue(group, symbol) {
  const n = PSDK_SYMBOLS[group]?.[symbol];
  if (n === undefined) throw new RangeError(`MissionToPsdkV2: unmapped symbol ${group}.${symbol}`);
  return n;
}

// Resolves a route attribute (wire number) to its PSDK firmware number via the
// catalog symbol: number → symbol (catalog) → number (PSDK table).
function psdkParamFromValue(group, value) {
  const symbol = categoryModel.symbolForValue(group, value);
  if (symbol == null) throw new RangeError(`MissionToPsdkV2: no symbol for ${group}=${value}`);
  return toPsdkValue(group, symbol);
}

// Internal numeric aliases (derived from the symbol tables) for the action
// builders below, which reference actions by short name.
const ACTION_TYPE = {
  stay: PSDK_SYMBOLS.actions.ACTION_STAY,
  photo: PSDK_SYMBOLS.actions.ACTION_PHOTO,
  video_start: PSDK_SYMBOLS.actions.ACTION_VIDEO_START,
  video_stop: PSDK_SYMBOLS.actions.ACTION_VIDEO_STOP,
  yaw: PSDK_SYMBOLS.actions.ACTION_YAW,
  gimbal: PSDK_SYMBOLS.actions.ACTION_GIMBAL,
  focus_camera: PSDK_SYMBOLS.actions.ACTION_FOCUS,
  zoom_camera: PSDK_SYMBOLS.actions.ACTION_ZOOM,
};

// DJI_WAYPOINT_V2_MISSION_GOTO_FIRST_WAYPOINT_MODE_*
const GOTO_FIRST_MODE = Object.freeze({
  SAFELY: 0, // Rise to first waypoint altitude first, then fly horizontally
  POINT_TO_POINT: 1, // Fly directly from current position to first waypoint
});

const TRIGGER = { ACTION_ASSOCIATED: 2, TRAJECTORY: 3, INTERVAL: 4, SAMPLE_REACH_POINT: 5 };
const TRIGGER_ASSOCIATED_TIMING = { SIMULTANEOUS: 1, FINISH: 2, UNKNOWN: 255 };
const ACTUATOR = { CAMERA: 1, GIMBAL: 2, AIRCRAFT: 4 };
const CAM_OP = { TAKE_PHOTO: 1, START_RECORD: 2, STOP_RECORD: 3, SET_FOCUS: 4, SET_ZOOM: 5 };
const GIMBAL_OP = { ROTATE: 1 };
const AIRCRAFT_OP = { ROTATE_YAW: 1, FLYING_CONTROL: 2 };
const AIRCRAFT_FLIGHT = { STOP: 0, START: 1 };

// ─── Route attribute defaults ─────────────────────────────────────────────────

// Defaults come from the mission_schema YAML, resolved per robot category (SSOT).
// route.attributes overrides any default the user explicitly set.
function extractRouteAttributes(routeAttributes, uavType) {
  const defaults = categoryModel.getAttributesDefaults(uavType);
  return { ...defaults, ...routeAttributes };
}

// ─── PSDK actuator builders ───────────────────────────────────────────────────

// WaypointV2CameraActuator fields: actuator_index, dji_waypoint_v2_action_actuator_camera_operation_type,
//   focus_param (WaypointV2CameraActuatorFocusParam), zoom_param (WaypointV2CameraActuatorFocalLengthParam)
function cameraActuator(operationType, { focusParam = {}, zoomParam = {} } = {}) {
  return {
    actuator_index: 0,
    dji_waypoint_v2_action_actuator_camera_operation_type: operationType,
    focus_param: { x: 0, y: 0, region_type: 0, width: 0, height: 0, ...focusParam },
    zoom_param: { focal_length: 0, ...zoomParam },
  };
}

// WaypointV2GimbalActuator fields: dji_waypoint_v2_action_actuator_gimbal_operation_type,
//   actuator_index, waypoint_v2_gimbal_actuator_rotation_param (WaypointV2GimbalActuatorRotationParam)
// WaypointV2GimbalActuatorRotationParam: x, y, z in 0.1° units; pitch absolute control
function gimbalActuator(pitchDeg) {
  return {
    dji_waypoint_v2_action_actuator_gimbal_operation_type: GIMBAL_OP.ROTATE,
    actuator_index: 0,
    waypoint_v2_gimbal_actuator_rotation_param: {
      x: 0,
      y: Math.round(pitchDeg * 10), // pitch in 0.1° units; absolute control
      z: 0,
      ctrl_mode: 0,
      roll_cmd_ignore: 1,
      pitch_cmd_ignore: 0,
      yaw_cmd_ignore: 1,
      abs_yaw_mode_ref: 0,
      duration_time: 10,
    },
  };
}

// WaypointV2AircraftControlActuator fields: actuator_index,
//   dji_waypoint_v2_action_actuator_aircraft_control_operation_type,
//   waypoint_v2_aircraft_control_actuator_flying (WaypointV2AircraftControlActuatorFlying),
//   waypoint_v2_aircraft_control_actuator_rotate_heading (WaypointV2AircraftControlActuatorRotateHeading)
function aircraftActuator({
  operationType = AIRCRAFT_OP.FLYING_CONTROL,
  isStartFlying = AIRCRAFT_FLIGHT.STOP,
  isRelative = 0,
  yaw = 0,
} = {}) {
  return {
    actuator_index: 0,
    dji_waypoint_v2_action_actuator_aircraft_control_operation_type: operationType,
    waypoint_v2_aircraft_control_actuator_flying: { is_start_flying: isStartFlying },
    waypoint_v2_aircraft_control_actuator_rotate_heading: { is_relative: isRelative, yaw },
  };
}

// ─── PSDK trigger builder ─────────────────────────────────────────────────────

// WaypointV2SampleReachPointTrigger fields: waypoint_index, terminate_num
// All other trigger types zeroed out (only SAMPLE_REACH_POINT is used)
function setTriggers(waypointIndex, actionIndex) {
  if (waypointIndex !== null) {
    return {
      waypoint_v2_action_trigger_type: TRIGGER.SAMPLE_REACH_POINT,
      waypoint_v2_sample_reach_point_trigger: { waypoint_index: waypointIndex, terminate_num: 0 },
    };
  }
  if (actionIndex !== null) {
    return {
      waypoint_v2_action_trigger_type: TRIGGER.ACTION_ASSOCIATED,
      waypoint_v2_associate_trigger: {
        action_associated_type: TRIGGER_ASSOCIATED_TIMING.FINISH,
        waiting_time: 0,
        action_id_associated: actionIndex,
      },
    };
  }
  return {
    waypoint_v2_associate_trigger: { action_associated_type: 0, waiting_time: 0, action_id_associated: 0 },
    waypoint_v2_interval_trigger: { start_index: 0, interval: 0, action_interval_type: 0 },
    waypoint_v2_trajectory_trigger: { start_index: 0, end_index: 0 },
    waypoint_v2_sample_reach_point_trigger: { waypoint_index: waypointIndex, terminate_num: 0 },
  };
}

// ─── Action resolvers (ACTION_TYPE → { waypoint_v2_action_actuator_type, actuators }) ─
// ACTION_TYPE uses psdk_interfaces hardcoded values — no DB lookup needed

const ACTION_RESOLVERS = {
  [ACTION_TYPE.photo]: () => ({
    waypoint_v2_action_actuator_type: ACTUATOR.CAMERA,
    waypoint_v2_camera_actuator: cameraActuator(CAM_OP.TAKE_PHOTO),
  }),
  [ACTION_TYPE.video_start]: () => ({
    waypoint_v2_action_actuator_type: ACTUATOR.CAMERA,
    waypoint_v2_camera_actuator: cameraActuator(CAM_OP.START_RECORD),
  }),
  [ACTION_TYPE.video_stop]: () => ({
    waypoint_v2_action_actuator_type: ACTUATOR.CAMERA,
    waypoint_v2_camera_actuator: cameraActuator(CAM_OP.STOP_RECORD),
  }),
  [ACTION_TYPE.gimbal]: (param) => ({
    waypoint_v2_action_actuator_type: ACTUATOR.GIMBAL,
    waypoint_v2_gimbal_actuator: gimbalActuator(param),
  }),
  [ACTION_TYPE.yaw]: (param) => ({
    waypoint_v2_action_actuator_type: ACTUATOR.AIRCRAFT,
    waypoint_v2_aircraft_control_actuator: aircraftActuator({
      operationType: AIRCRAFT_OP.ROTATE_YAW,
      yaw: param,
    }),
  }),
  [ACTION_TYPE.focus_camara]: (param) => ({
    waypoint_v2_action_actuator_type: ACTUATOR.CAMERA,
    waypoint_v2_camera_actuator: cameraActuator(CAM_OP.SET_FOCUS, {
      focusParam: { x: param, y: param, region_type: 0, width: 0, height: 0 },
    }),
  }),
  [ACTION_TYPE.zoom_camera]: (param) => ({
    waypoint_v2_action_actuator_type: ACTUATOR.CAMERA,
    waypoint_v2_camera_actuator: cameraActuator(CAM_OP.SET_ZOOM, {
      zoomParam: { focal_length: param },
    }),
  }),
  [ACTION_TYPE.stay]: (isStartFlying = AIRCRAFT_FLIGHT.STOP) => ({
    waypoint_v2_action_actuator_type: ACTUATOR.AIRCRAFT,
    waypoint_v2_aircraft_control_actuator: aircraftActuator({
      operationType: AIRCRAFT_OP.FLYING_CONTROL,
      isStartFlying,
    }),
  }),
};

// ─── PSDK action list builder ─────────────────────────────────────────────────

// Emits one action, returns its assigned id.
function pushAction(actions, trigger, resolver) {
  const id = actions.length;
  actions.push({ action_id: id, ...trigger, ...resolver });
  return id;
}

// Builds the ordered action chain for a single waypoint following PSDK sequencing rules:
//
//   [1] StopFlying         ← trigger: reach WP     (only if stay)
//   [2] StartRecording?    ← trigger: reach WP     (only if video_start)
//   [3] RotateYaw          ← trigger: after [2] or [1]
//   [4] RotateGimbal?      ← trigger: after [3]    (only if gimbal != 0)
//        flight_anchor_id  = last of yaw / gimbal
//   [5] StopRecording?     ← trigger: after flight_anchor  (if recording && take_photo)
//   [5b] SetZoom?          ← trigger: after [5] or flight_anchor  (only if zoom_camera)
//   [5c] SetFocus?         ← trigger: after [5b] or [5] or flight_anchor  (only if focus_camara)
//   [6] TakePhoto?         ← trigger: after [5c/5b/5] or flight_anchor
//   [7] StartRecording?    ← trigger: after [6]    (if was recording and took photo)
//   [8] StartFlying        ← trigger: after flight_anchor_id  (NEVER after camera)
//
function buildActionsForWaypoint(actions, wpIdx, item, isRecording) {
  const act = item.action;
  const hasStay = Object.prototype.hasOwnProperty.call(act, 'stay') || true; // default is stay, even if not present
  const hasYaw = Object.prototype.hasOwnProperty.call(act, 'yaw');
  const hasGimbal = Object.prototype.hasOwnProperty.call(act, 'gimbal') && Number(act.gimbal) !== 0;
  const hasTakePhoto = Object.prototype.hasOwnProperty.call(act, 'photo');
  const hasVideoStart = Object.prototype.hasOwnProperty.call(act, 'video_start');
  const hasVideoStop = Object.prototype.hasOwnProperty.call(act, 'video_stop');
  const hasZoom = Object.prototype.hasOwnProperty.call(act, 'zoom_camera');
  const hasFocus = Object.prototype.hasOwnProperty.call(act, 'focus_camara');

  // A wp with no meaningful actions to sequence can be skipped
  if (!hasStay && !hasYaw && !hasGimbal && !hasTakePhoto && !hasVideoStart && !hasVideoStop && !hasZoom && !hasFocus)
    return;

  const wpTrigger = setTriggers(wpIdx, null);

  // [1] StopFlying — triggered by reaching the waypoint
  let prevId = null;
  if (hasStay) {
    prevId = pushAction(actions, wpTrigger, ACTION_RESOLVERS[ACTION_TYPE.stay]());
  }

  // [2] StartRecording — triggered by reaching the waypoint (independent of StopFlying)
  if (hasVideoStart) {
    pushAction(actions, wpTrigger, ACTION_RESOLVERS[ACTION_TYPE.video_start]());
    // prevId stays as StopFlying (or null) — yaw chains after stay, not after recording start
  }

  // [3] RotateYaw — chains after [1], or triggered by WP if no stay
  const yawTrigger = prevId !== null ? setTriggers(null, prevId) : wpTrigger;
  let flightAnchorId;
  if (hasYaw) {
    flightAnchorId = pushAction(actions, yawTrigger, ACTION_RESOLVERS[ACTION_TYPE.yaw](Number(act.yaw ?? 0)));
  } else {
    // No yaw: flight_anchor starts at the same point as yaw would (prevId or WP)
    flightAnchorId = prevId;
  }

  // [4] RotateGimbal — chains after [3]
  if (hasGimbal) {
    const gimbalTrigger = flightAnchorId !== null ? setTriggers(null, flightAnchorId) : wpTrigger;
    flightAnchorId = pushAction(actions, gimbalTrigger, ACTION_RESOLVERS[ACTION_TYPE.gimbal](Number(act.gimbal)));
  }

  // flight_anchor_id is now set — camera actions chain from here, StartFlying also uses it
  const anchorTrigger = flightAnchorId !== null ? setTriggers(null, flightAnchorId) : wpTrigger;
  let cameraChainId = flightAnchorId;

  // [5] StopRecording — two cases:
  //   a) explicit video_stop without photo → emit directly, standalone
  //   b) recording active (prior WP or this WP) AND taking photo → stop before photo, resume after
  const recordingActiveAtPhoto = hasTakePhoto && (isRecording || hasVideoStart);
  const needsPhotoWithRecording = recordingActiveAtPhoto;
  if ((hasVideoStop && !hasTakePhoto) || recordingActiveAtPhoto) {
    const stopTrigger = cameraChainId !== null ? setTriggers(null, cameraChainId) : anchorTrigger;
    cameraChainId = pushAction(actions, stopTrigger, ACTION_RESOLVERS[ACTION_TYPE.video_stop]());
  }

  // [5b] SetZoom — chains after [5] or flight_anchor
  if (hasZoom) {
    const zoomTrigger = cameraChainId !== null ? setTriggers(null, cameraChainId) : anchorTrigger;
    cameraChainId = pushAction(
      actions,
      zoomTrigger,
      ACTION_RESOLVERS[ACTION_TYPE.zoom_camera](Number(act.zoom_camera))
    );
  }

  // [5c] SetFocus — chains after [5b] or [5] or flight_anchor
  if (hasFocus) {
    const focusTrigger = cameraChainId !== null ? setTriggers(null, cameraChainId) : anchorTrigger;
    cameraChainId = pushAction(
      actions,
      focusTrigger,
      ACTION_RESOLVERS[ACTION_TYPE.focus_camara](Number(act.focus_camara))
    );
  }

  // [6] TakePhoto — chains after [5c/5b/5] or flight_anchor
  if (hasTakePhoto) {
    const photoTrigger = cameraChainId !== null ? setTriggers(null, cameraChainId) : anchorTrigger;
    cameraChainId = pushAction(actions, photoTrigger, ACTION_RESOLVERS[ACTION_TYPE.photo]());
  }

  // [7] StartRecording — resumes recording if it was active when the photo was taken
  if (needsPhotoWithRecording) {
    const resumeTrigger = cameraChainId !== null ? setTriggers(null, cameraChainId) : anchorTrigger;
    pushAction(actions, resumeTrigger, ACTION_RESOLVERS[ACTION_TYPE.video_start]());
  }

  // [8] StartFlying — always chains after the last action (yaw/gimbal/camera) or the waypoint if no actions
  if (hasStay) {
    const startTrigger = cameraChainId !== null ? setTriggers(null, cameraChainId) : wpTrigger;
    pushAction(actions, startTrigger, ACTION_RESOLVERS[ACTION_TYPE.stay](AIRCRAFT_FLIGHT.START));
  }
}

function buildPsdkActions(waypoints) {
  const actions = [];
  let isRecording = false;
  for (let wpIdx = 0; wpIdx < waypoints.length; wpIdx++) {
    const item = waypoints[wpIdx];
    if (!Object.prototype.hasOwnProperty.call(item, 'action')) continue;
    buildActionsForWaypoint(actions, wpIdx, item, isRecording);
    // update recording state for next waypoints
    if (Object.prototype.hasOwnProperty.call(item.action, 'video_start')) isRecording = true;
    if (Object.prototype.hasOwnProperty.call(item.action, 'video_stop')) isRecording = false;
  }
  return actions;
}

// ─── PSDK parameter validation ────────────────────────────────────────────────

// All modes (mode_yaw/mode_trace/mode_landing/mode_turn) are validated implicitly
// by psdkParamFromValue — an unmapped or unknown symbol throws RangeError.

// ─── Encoder: route crudo → psdk_interfaces/srv/InitWaypointV2Setting ────────

export function MissionToPsdkV2(route, { missionId = 1 } = {}) {
  const { idle_vel, max_vel, mode_yaw, mode_trace, mode_landing } = extractRouteAttributes(
    route.attributes ?? {},
    route.uav_type
  );

  // Resolve each route mode from its wire number to the PSDK firmware number,
  // going through the catalog symbol (number → symbol → firmware number).
  const headingMode = psdkParamFromValue('mode_yaw', mode_yaw);
  const traceMode = psdkParamFromValue('mode_trace', mode_trace);
  const finishAction = psdkParamFromValue('mode_landing', mode_landing);

  // mode_turn is per-waypoint: each wp may carry its own wp.mode_turn (wire number);
  // fall back to the catalog default when absent. Resolved via symbol like the rest.
  const defaultTurn = categoryModel.getWaypointDefault(route.uav_type, 'mode_turn');

  const waypoints = Object.values(route.wp);

  // Wire value for TURN_MODE_AUTO is 0 (mission_schema.yaml — NOT the PSDK firmware number);
  // when a waypoint carries this value we pick CW or CCW at encode-time to minimise the arc.
  const WIRE_TURN_AUTO = 0;
  const PSDK_CW = PSDK_SYMBOLS.mode_turn.TURN_MODE_CLOCKWISE;
  const PSDK_CCW = PSDK_SYMBOLS.mode_turn.TURN_MODE_COUNTER_CLOCKWISE;

  // Returns the signed shortest delta in [-180, 180] between two yaw angles.
  function yawDelta(from, to) {
    return ((to - from + 540) % 360) - 180;
  }

  // Tracks the last action yaw seen across waypoints for arc-minimisation.
  let prevActionYaw = null;

  const mission = waypoints.map((item, i) => {
    const wireTurn = item.mode_turn ?? defaultTurn;
    let turn_mode = PSDK_CW;
    if (wireTurn === WIRE_TURN_AUTO) {
      const actionYaw = item.action?.yaw != null ? Number(item.action.yaw) : null;
      if (actionYaw !== null) {
        // Compare this action yaw against the last seen action yaw (or wp heading if none yet).
        const from = prevActionYaw ?? (i > 0 ? (waypoints[i - 1].yaw ?? 0) : 0);
        turn_mode = yawDelta(from, actionYaw) >= 0 ? PSDK_CW : PSDK_CCW;
        prevActionYaw = actionYaw;
      } else if (headingMode === PSDK_SYMBOLS.mode_yaw.HEADING_MODE_WAYPOINT_CUSTOM) {
        const prevYaw = i > 0 ? (waypoints[i - 1].yaw ?? 0) : 0;
        const curYaw = item.yaw ?? 0;
        turn_mode = yawDelta(prevYaw, curYaw) >= 0 ? PSDK_CW : PSDK_CCW;
      }
    } else {
      turn_mode = psdkParamFromValue('mode_turn', wireTurn);
    }
    if (turn_mode !== PSDK_CW && turn_mode !== PSDK_CCW) {
      throw new RangeError(`MissionToPsdkV2: invalid turn_mode ${turn_mode} at waypoint ${i}`);
    }
    return {
      longitude: item.pos[1],
      latitude: item.pos[0],
      relative_height: item.pos[2],
      waypoint_type: traceMode,
      heading_mode: headingMode,
      config: {
        // use per-waypoint speed only when it differs from idle
        use_local_cruise_vel: Object.prototype.hasOwnProperty.call(item, 'speed') && item.speed !== idle_vel ? 1 : 0,
        use_local_max_vel: 0,
      },
      damping_distance: 0,
      heading: item.yaw ?? 0,
      turn_mode,
      position_x: 0,
      position_y: 0,
      position_z: 0,
      max_flight_speed: max_vel,
      auto_flight_speed: item.speed ?? idle_vel,
    };
  });

  const actions = buildPsdkActions(waypoints);

  return {
    waypoint_v2_init_settings: {
      mission_id: missionId,
      miss_total_len: waypoints.length,
      repeat_times: 0,
      finished_action: finishAction,
      max_flight_speed: max_vel,
      auto_flight_speed: idle_vel,
      exit_mission_on_signal_lost: 1,
      goto_first_waypoint_mode: GOTO_FIRST_MODE.SAFELY,
      mission,
      actions,
    },
    polygon_num: 0,
    radius: 0,
    action_num: actions.length,
    upload_after_init: true,
  };
}
