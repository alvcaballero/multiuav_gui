// Canonical vehicle state the server puts under `position.attributes`: `armState`,
// `flightPhase`, `flightMode`, only for a VTOL `vtolState`, and where the vehicle reports them the control
// authority and its own mission. Mirror of ARM_STATE / FLIGHT_PHASE / FLIGHT_MODE / VTOL_STATE /
// CONTROL_SOURCE / MISSION_* in server/config/status.js; server/test/test-vehicle-state-mirror.js
// fails if they drift apart.
// The server never sends a firmware label, so the text the UI shows lives here.
export const ARM_STATE = Object.freeze({
  ARMED: 'ARMED',
  DISARMED: 'DISARMED',
  UNKNOWN: 'UNKNOWN',
});

export const FLIGHT_PHASE = Object.freeze({
  ON_GROUND: 'ON_GROUND',
  TAKEOFF: 'TAKEOFF',
  IN_AIR: 'IN_AIR',
  LANDING: 'LANDING',
  UNKNOWN: 'UNKNOWN',
});

export const VTOL_STATE = Object.freeze({
  MC: 'MC',
  FW: 'FW',
  TRANSITION_TO_FW: 'TRANSITION_TO_FW',
  TRANSITION_TO_MC: 'TRANSITION_TO_MC',
  UNKNOWN: 'UNKNOWN',
});

export const FLIGHT_MODE = Object.freeze({
  MANUAL: 'MANUAL',
  ASSISTED: 'ASSISTED',
  HOLD: 'HOLD',
  MISSION: 'MISSION',
  TAKEOFF: 'TAKEOFF',
  LAND: 'LAND',
  RTL: 'RTL',
  AUTO_OTHER: 'AUTO_OTHER',
  OFFBOARD: 'OFFBOARD',
  EMERGENCY: 'EMERGENCY',
  OTHER: 'OTHER',
  UNKNOWN: 'UNKNOWN',
});

// What the vehicle answered to a command. Mirror of COMMAND_RESULT in server/config/status.js.
export const COMMAND_RESULT = Object.freeze({
  ACCEPTED: 'ACCEPTED',
  REJECTED: 'REJECTED',
  DENIED: 'DENIED',
  UNSUPPORTED: 'UNSUPPORTED',
  FAILED: 'FAILED',
  IN_PROGRESS: 'IN_PROGRESS',
  CANCELLED: 'CANCELLED',
  UNKNOWN: 'UNKNOWN',
});

const RESULT_LABEL = {
  [COMMAND_RESULT.ACCEPTED]: 'accepted',
  [COMMAND_RESULT.REJECTED]: 'temporarily rejected',
  [COMMAND_RESULT.DENIED]: 'denied',
  [COMMAND_RESULT.UNSUPPORTED]: 'unsupported',
  [COMMAND_RESULT.FAILED]: 'failed',
  [COMMAND_RESULT.IN_PROGRESS]: 'in progress',
  [COMMAND_RESULT.CANCELLED]: 'cancelled',
};

const ARM_LABEL = {
  [ARM_STATE.ARMED]: 'Armed',
  [ARM_STATE.DISARMED]: 'Disarmed',
};

const PHASE_LABEL = {
  [FLIGHT_PHASE.ON_GROUND]: 'On ground',
  [FLIGHT_PHASE.TAKEOFF]: 'Taking off',
  [FLIGHT_PHASE.IN_AIR]: 'In air',
  [FLIGHT_PHASE.LANDING]: 'Landing',
};

const VTOL_LABEL = {
  [VTOL_STATE.MC]: 'MC',
  [VTOL_STATE.FW]: 'FW',
  [VTOL_STATE.TRANSITION_TO_FW]: 'MC → FW',
  [VTOL_STATE.TRANSITION_TO_MC]: 'FW → MC',
};

const MODE_LABEL = {
  [FLIGHT_MODE.MANUAL]: 'Manual',
  [FLIGHT_MODE.ASSISTED]: 'Assisted',
  [FLIGHT_MODE.HOLD]: 'Hold',
  [FLIGHT_MODE.MISSION]: 'Mission',
  [FLIGHT_MODE.TAKEOFF]: 'Takeoff',
  [FLIGHT_MODE.LAND]: 'Land',
  [FLIGHT_MODE.RTL]: 'Return',
  [FLIGHT_MODE.AUTO_OTHER]: 'Auto',
  [FLIGHT_MODE.OFFBOARD]: 'Offboard',
  [FLIGHT_MODE.EMERGENCY]: 'Emergency',
  [FLIGHT_MODE.OTHER]: 'Other',
};

// null when the source says nothing (UNKNOWN, or the attribute is absent): the caller decides
// what to show, a dash in a stat, nothing in a caption.
// Who holds the control authority (`controlSource` + `controlAuthority`, DJI control_mode).
// Mirror of CONTROL_SOURCE in server/config/status.js.
export const CONTROL_SOURCE = Object.freeze({
  PILOT: 'PILOT',
  MOBILE_APP: 'MOBILE_APP',
  ONBOARD: 'ONBOARD',
  UNKNOWN: 'UNKNOWN',
});

// The mission the vehicle runs on its own (`missionExecState`, DJI WaypointV2) and what happened to it
// (`missionEvents`). Mirrors of MISSION_EXEC_STATE / MISSION_EVENT / MISSION_STOP_REASON.
export const MISSION_EXEC_STATE = Object.freeze({
  IDLE: 'IDLE',
  PREPARED: 'PREPARED',
  STARTING: 'STARTING',
  RUNNING: 'RUNNING',
  PAUSED: 'PAUSED',
  RESUMING: 'RESUMING',
  EXITING: 'EXITING',
  UNKNOWN: 'UNKNOWN',
});

export const MISSION_EVENT = Object.freeze({
  INTERRUPTED: 'INTERRUPTED',
  RESUMED: 'RESUMED',
  STOPPED: 'STOPPED',
  WP_REACHED: 'WP_REACHED',
  FINISHED: 'FINISHED',
  AVOIDING_OBSTACLE: 'AVOIDING_OBSTACLE',
  OTHER: 'OTHER',
});

export const MISSION_STOP_REASON = Object.freeze({
  COMPLETED: 'COMPLETED',
  USER: 'USER',
  MOTORS_STARTING: 'MOTORS_STARTING',
  TAKEOFF_TIMEOUT: 'TAKEOFF_TIMEOUT',
  TAKEOFF_ALTITUDE: 'TAKEOFF_ALTITUDE',
  TAKEOFF_ABNORMAL: 'TAKEOFF_ABNORMAL',
  PREEMPTED: 'PREEMPTED',
  PATH_PLANNING: 'PATH_PLANNING',
  UNKNOWN: 'UNKNOWN',
});

const CONTROL_LABEL = {
  [CONTROL_SOURCE.PILOT]: 'RC',
  [CONTROL_SOURCE.MOBILE_APP]: 'App',
  [CONTROL_SOURCE.ONBOARD]: 'Onboard',
};

const MISSION_EXEC_LABEL = {
  [MISSION_EXEC_STATE.IDLE]: 'Idle',
  [MISSION_EXEC_STATE.PREPARED]: 'Prepared',
  [MISSION_EXEC_STATE.STARTING]: 'Starting',
  [MISSION_EXEC_STATE.RUNNING]: 'Running',
  [MISSION_EXEC_STATE.PAUSED]: 'Paused',
  [MISSION_EXEC_STATE.RESUMING]: 'Resuming',
  [MISSION_EXEC_STATE.EXITING]: 'Exiting',
};

const STOP_REASON_LABEL = {
  [MISSION_STOP_REASON.COMPLETED]: 'completed',
  [MISSION_STOP_REASON.USER]: 'stopped from outside',
  [MISSION_STOP_REASON.MOTORS_STARTING]: 'motors were starting',
  [MISSION_STOP_REASON.TAKEOFF_TIMEOUT]: 'takeoff timed out',
  [MISSION_STOP_REASON.TAKEOFF_ALTITUDE]: 'takeoff below its height',
  [MISSION_STOP_REASON.TAKEOFF_ABNORMAL]: 'takeoff failed',
  [MISSION_STOP_REASON.PREEMPTED]: 'preempted (joystick control)',
  [MISSION_STOP_REASON.PATH_PLANNING]: 'route not plannable',
  [MISSION_STOP_REASON.UNKNOWN]: 'unknown reason',
};

const EVENT_LABEL = {
  [MISSION_EVENT.INTERRUPTED]: (e) => (e.reasonRaw === 0 ? 'Paused by the RC' : 'Interrupted'),
  [MISSION_EVENT.RESUMED]: () => 'Resumed',
  [MISSION_EVENT.STOPPED]: (e) =>
    `Stopped: ${STOP_REASON_LABEL[e.reason] ?? STOP_REASON_LABEL.UNKNOWN}`,
  [MISSION_EVENT.WP_REACHED]: (e) => `WP ${e.wp} reached`,
  [MISSION_EVENT.FINISHED]: () => 'Finished',
  [MISSION_EVENT.AVOIDING_OBSTACLE]: () => 'Avoiding an obstacle',
};

export const armLabel = (armState) => ARM_LABEL[armState] ?? null;
export const phaseLabel = (flightPhase) => PHASE_LABEL[flightPhase] ?? null;
export const modeLabel = (flightMode) => MODE_LABEL[flightMode] ?? null;
export const vtolLabel = (vtolState) => VTOL_LABEL[vtolState] ?? null;
export const resultLabel = (result) => RESULT_LABEL[result] ?? null;
export const controlLabel = (controlSource) => CONTROL_LABEL[controlSource] ?? null;
export const missionExecLabel = (state) => MISSION_EXEC_LABEL[state] ?? null;
/** One entry of `attributes.missionEvents` ({kind, reason?, wp?}) as a line of text. */
export const missionEventLabel = (event) => EVENT_LABEL[event?.kind]?.(event) ?? null;
/** A stop before the end: what the operator must notice in the timeline. */
export const isAbnormalStop = (event) =>
  event?.kind === MISSION_EVENT.STOPPED && event.reason !== MISSION_STOP_REASON.COMPLETED;
