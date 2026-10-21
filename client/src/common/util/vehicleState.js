// Canonical vehicle state the server puts under `position.attributes`: `armState`,
// `flightPhase` and `flightMode`. Mirror of ARM_STATE / FLIGHT_PHASE / FLIGHT_MODE in
// server/config/status.js; server/test/test-vehicle-state-mirror.js fails if they drift apart.
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
export const armLabel = (armState) => ARM_LABEL[armState] ?? null;
export const phaseLabel = (flightPhase) => PHASE_LABEL[flightPhase] ?? null;
export const modeLabel = (flightMode) => MODE_LABEL[flightMode] ?? null;
export const resultLabel = (result) => RESULT_LABEL[result] ?? null;
