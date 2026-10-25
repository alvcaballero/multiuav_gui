import { round } from '../../common/utils.js';
import { ARM_STATE, FLIGHT_MODE, FLIGHT_PHASE } from '../../config/status.js';

// Pure merge rules for one telemetry patch into one cached position. No controller
// imports, so it runs in a test without opening the database: side effects (events)
// are RETURNED by `applyPatch`, and `positionsModel` is the one that executes them.

/**
 * A telemetry patch: what `positionsModel.updatePosition` accepts. It is a contract
 * shared by every producer (`msgDecode.js`, `FlatbufferServer.js`, `commands.js`) and a
 * patch carries only the keys of ONE message, so absent keys mean "unchanged".
 * Groups travel together: a key that triggers a reducer is listed first.
 *
 * @typedef {object} TelemetryPatch
 * @property {number|string} deviceId
 * @property {number} [latitude] - with `longitude` and `deviceTime`
 * @property {number} [longitude]
 * @property {string} [deviceTime]
 * @property {number} [altitude]
 * @property {string} [positionSource] - slot that produced lat/lon; set by `decodeMsg` only when the entry declares `priority`/`fresh_ms`
 * @property {number} [positionPriority] - higher wins (default 0)
 * @property {string} [positionAttribute] - key of `attributes` that keeps the last fix of this source (default: the slot)
 * @property {number} [positionFreshMs] - how long a patch of this source blocks lower priorities (default POSITION_FRESH_MS)
 * @property {number} [course] - degrees, 0 = north
 * @property {number} [speed] - m/s
 * @property {number|string} [batteryLevel] - percent; rounded on write
 * @property {{x:number,y:number,z?:number}} [gimbal] - pitch, roll, yaw (deg, absolute); no z keeps the cached yaw
 * @property {{down?:number,front?:number,right?:number,back?:number,left?:number,up?:number}} [obstacle_info]
 * @property {boolean} [setHome] - snapshot the current lat/lon/alt as home
 * @property {number[]} [home] - [lat, lon, alt] the vehicle reports as its home (MAVLink HOME_POSITION)
 * @property {{x:number,y:number,z:number}} [localposition] - local ENU, meters
 * @property {string} [armState] - ARM_STATE (config/status.js), already in the GCS vocabulary
 * @property {string} [flightMode] - FLIGHT_MODE, how it is being flown
 * @property {number|string} [flightModeRaw] - the firmware's own mode code, for debugging
 * @property {string} [flightPhase] - FLIGHT_PHASE, where it is
 * @property {number|string} [flightPhaseRaw] - the firmware's own phase code, for debugging
 * @property {string} [vtolState] - VTOL_STATE, only for a VTOL (MC / FW / transitioning)
 * @property {number} [vtolStateRaw] - the firmware's own VTOL state code, for debugging
 * @property {boolean} [failsafe]
 * @property {boolean} [flightCheck]
 * @property {number} [systemId] - PX4 MAV_SYS_ID (vehicleCommand.js targets it)
 * @property {string} [commandAck] - with `resultCmdAck`; the command's name, informational
 * @property {number} [commandAckRaw] - the firmware's own command id
 * @property {string} [resultCmdAck] - COMMAND_RESULT (config/status.js)
 * @property {number} [resultCmdAckRaw] - the firmware's own result code
 * @property {string} [uav_state] - with `mission_state`, `wp_reached`
 * @property {string} [mission_state]
 * @property {string} [wp_reached]
 * @property {ArrayLike<number>} [sensors_humidity] - six channels, see HUMIDITY_CHANNELS
 * @property {number|boolean} [threat] - 2 = threat, 3 = confirmed, anything else clears
 */

// Order of the cached `obstacle_info` array.
const OBSTACLE_SIDES = ['down', 'front', 'right', 'back', 'left', 'up'];

const HUMIDITY_CHANNELS =['MIC_1', 'MIC_2', 'MIC_3', 'Metano', 'Alcohol', 'CO'];

const alarmFor = (threat) => (threat == 2 ? 'threat' : threat == 3 ? 'confirm' : 'None');
const THREAT_MESSAGE = { threat: 'Detected a threat', confirm: 'Confirmed a threat' };

export function createDefaultPosition(deviceId) {
  return {
    deviceId,
    accuracy: 0.0,
    speed: 0.0,
    course: 0.0,
    attributes: {
      batteryLevel: 0,
      gimbal: [0, 0, 0],
      obstacle_info: [100, 100, 100, 100, 100, 100],
      takeoff_height: 400,
      mission_state: 'Ready',
      wp_reached: null,
      uav_state: 'OK',
      armState: ARM_STATE.UNKNOWN,
      flightMode: FLIGHT_MODE.UNKNOWN,
      flightPhase: FLIGHT_PHASE.UNKNOWN,
      alarm: 'UNDEFINED',
    },
  };
}

const has = (p, key) => Object.hasOwn(p, key);

// ─── position source arbitration ───────────────────────────────────────────────
// A category may subscribe SEVERAL slots that carry lat/lon (DJI: `position` GNSS and
// `position_rtk`). Each entry declares `priority` (higher wins, default 0) and `fresh_ms`
// in devices_msg.yaml; `decodeMsg` stamps them on the patch. A lat/lon patch is dropped
// while another source of HIGHER priority delivered one within ITS `fresh_ms`: when that
// source goes quiet the lower one takes over again, and when it comes back it wins at
// once. Patches with no `positionSource` (every other category) are never arbitrated.
// Every source also keeps its OWN last fix in `attributes[<entry.attribute ?? slot>]`
// (`{ latitude, longitude, altitude, at, fix }`, `at` = server ms of its last message), winner or not,
// so the client can show both. `fix: 'nofix'` = the receiver sent 0,0 (no latitude/longitude then).
export const POSITION_FRESH_MS = 3000;

const sourcesSeen = new WeakMap(); // position -> Map(slot -> { priority, freshMs, at })

const isNullIsland = (p) => p.latitude === 0 && p.longitude === 0;

/**
 * Decides whether the lat/lon of `patch` is applied. When it is not, the position keys of the
 * patch (latitude, longitude, deviceTime and the altitude of the same message) are DELETED
 * from it: the caller then sees a patch with no position (the controller does not announce it).
 */
function gatePosition(position, patch, now) {
  if (!has(patch, 'latitude')) return;
  const drop = () => {
    for (const key of ['latitude', 'longitude', 'altitude', 'deviceTime']) delete patch[key];
  };
  const slot = patch.positionSource;
  const key = patch.positionAttribute ?? slot;

  // 0,0 is what a NavSatFix carries before the receiver has a fix: not a position, not a sign of life.
  if (isNullIsland(patch)) {
    if (slot !== undefined) position.attributes[key] = { at: now, fix: 'nofix' };
    return drop();
  }
  if (slot === undefined) return;
  position.attributes[key] = {
    latitude: patch.latitude,
    longitude: patch.longitude,
    altitude: patch.altitude,
    at: now,
    fix: 'ok',
  };

  const priority = patch.positionPriority ?? 0;
  const seen = sourcesSeen.get(position) ?? sourcesSeen.set(position, new Map()).get(position);
  seen.set(slot, { priority, freshMs: patch.positionFreshMs ?? POSITION_FRESH_MS, at: now });

  for (const [other, s] of seen) {
    if (other !== slot && s.priority > priority && now - s.at <= s.freshMs) return drop();
  }

  // Attribute only, no Event: near towers the RTK fix comes and goes, and an Event is a DB row plus a
  // snackbar per switch. The hysteresis that would calm the switching itself is open in TODO.md.
  position.attributes.positionSource = slot;
}

const VEHICLE_STATE_KEYS = [
  'armState',
  'flightMode',
  'flightModeRaw',
  'flightPhase',
  'flightPhaseRaw',
  'vtolState',
  'vtolStateRaw',
  'failsafe',
  'flightCheck',
  'systemId',
];

// Order matters where one reducer reads what an earlier one wrote (`setHome` reads
// latitude/longitude/altitude). `apply(position, patch, effects)` mutates `position`
// and may push events to `effects`.
export const REDUCERS = [
  {
    when: (p) => has(p, 'latitude'),
    apply: (pos, p) => {
      pos.latitude = p.latitude;
      pos.longitude = p.longitude;
      pos.deviceTime = p.deviceTime;
    },
  },
  { when: (p) => has(p, 'altitude'), apply: (pos, p) => (pos.altitude = p.altitude) },
  { when: (p) => has(p, 'course'), apply: (pos, p) => (pos.course = p.course) },
  { when: (p) => has(p, 'speed'), apply: (pos, p) => (pos.speed = p.speed) },
  {
    when: (p) => has(p, 'batteryLevel'),
    apply: (pos, p) => (pos.attributes.batteryLevel = Math.round(Number.parseFloat(p.batteryLevel))),
  },
  {
    when: (p) => has(p, 'gimbal'),
    apply: (pos, p) => {
      const yaw = p.gimbal.z == null ? pos.attributes.gimbal?.[2] ?? 0 : round(p.gimbal.z, 1);
      pos.attributes.gimbal = [round(p.gimbal.x, 1), round(p.gimbal.y, 1), yaw];
    },
  },
  {
    when: (p) => has(p, 'obstacle_info'),
    // A sensor may report only some sides (a downward sonar gives just `down`): the
    // others keep their last value instead of turning into NaN.
    apply: (pos, p) => {
      const o = p.obstacle_info;
      pos.attributes.obstacle_info = OBSTACLE_SIDES.map((side, i) =>
        o?.[side] == null ? pos.attributes.obstacle_info[i] : round(o[side], 1)
      );
    },
  },
  {
    when: (p) => has(p, 'setHome'),
    apply: (pos) => (pos.attributes.home = [pos.latitude, pos.longitude, pos.altitude]),
  },
  { when: (p) => has(p, 'home'), apply: (pos, p) => (pos.attributes.home = [...p.home]) },
  {
    when: (p) => has(p, 'localposition'),
    apply: (pos, p) => {
      pos.attributes.localposition = [round(p.localposition.x, 2), round(p.localposition.y, 2), round(p.localposition.z, 2)];
      pos.deviceTime = new Date().toISOString();
    },
  },
  {
    // Each source carries a different subset (PX4 vehicle_status all of them, DJI flight_status
    // arm + phase), so only the keys present are written, and a null never wipes what is cached.
    when: (p) => VEHICLE_STATE_KEYS.some((key) => has(p, key)),
    apply: (pos, p) => {
      for (const key of VEHICLE_STATE_KEYS) if (p[key] != null) pos.attributes[key] = p[key];
    },
  },
  {
    when: (p) => has(p, 'commandAck'),
    apply: (pos, p) => {
      pos.attributes.commandAck = p.commandAck;
      pos.attributes.commandAckRaw = p.commandAckRaw;
      pos.attributes.resultCmdAck = p.resultCmdAck;
      pos.attributes.resultCmdAckRaw = p.resultCmdAckRaw;
    },
  },
  {
    when: (p) => has(p, 'uav_state'),
    apply: (pos, p) => {
      pos.attributes.mission_state = p.mission_state;
      pos.attributes.wp_reached = p.wp_reached;
      pos.attributes.uav_state = p.uav_state;
    },
  },
  {
    when: (p) => has(p, 'sensors_humidity'),
    apply: (pos, p) => {
      // 0 is a valid reading; only a channel the message does not carry is skipped.
      HUMIDITY_CHANNELS.forEach((channel, i) => {
        if (p.sensors_humidity[i] != null) pos.attributes[channel] = p.sensors_humidity[i];
      });
    },
  },
  {
    when: (p) => has(p, 'threat'),
    apply: (pos, p, effects) => {
      const next = alarmFor(p.threat);
      if (next !== 'None' && pos.attributes.alarm !== next) {
        effects.push({
          type: 'warning',
          deviceId: p.deviceId,
          attributes: {
            action: 'Thread',
            message: THREAT_MESSAGE[next],
            // Where the vehicle IS, not what this message carries: a threat message has no
            // position of its own. Earlier reducers already applied the patch's, if any.
            positions: { latitude: pos.latitude, longitude: pos.longitude, altitude: pos.altitude },
          },
        });
      }
      pos.attributes.alarm = next;
    },
  },
];

/**
 * Merges `patch` into `position` (in place) and returns the events to emit. A lat/lon patch
 * that loses the source arbitration (or is 0,0) has its position keys removed from `patch`.
 * @param {object} position
 * @param {TelemetryPatch} patch
 * @param {number} [now] - ms; injected so a test can move the clock
 * @returns {object[]} events for `eventsController.addEvent`
 */
export function applyPatch(position, patch, now = Date.now()) {
  const effects = [];
  gatePosition(position, patch, now);
  for (const reducer of REDUCERS) {
    if (reducer.when(patch)) reducer.apply(position, patch, effects);
  }
  return effects;
}
