import { round } from '../../common/utils.js';

// Pure merge rules for one telemetry patch into one cached position. No controller
// imports, so it runs in a test without opening the database: side effects (events)
// are RETURNED by `applyPatch`, and `positionsModel` is the one that executes them.

/**
 * A telemetry patch: what `positionsModel.updatePosition` accepts. It is a contract
 * shared by every producer (`rosDecode.js`, `FlatbufferServer.js`, `commands.js`) and a
 * patch carries only the keys of ONE message, so absent keys mean "unchanged".
 * Groups travel together: a key that triggers a reducer is listed first.
 *
 * @typedef {object} TelemetryPatch
 * @property {number|string} deviceId
 * @property {number} [latitude] - with `longitude` and `deviceTime`
 * @property {number} [longitude]
 * @property {string} [deviceTime]
 * @property {number} [altitude]
 * @property {number} [course] - degrees, 0 = north
 * @property {number} [speed] - m/s
 * @property {number|string} [batteryLevel] - percent; rounded on write
 * @property {{x:number,y:number,z:number}} [gimbal]
 * @property {{down?:number,front?:number,right?:number,back?:number,left?:number,up?:number}} [obstacle_info]
 * @property {boolean} [setHome] - snapshot the current lat/lon/alt as home
 * @property {{x:number,y:number,z:number}} [localposition] - local ENU, meters
 * @property {string} [armState] - with `navState`, `failsafe`, `flightCheck`
 * @property {string} [navState]
 * @property {boolean} [failsafe]
 * @property {boolean} [flightCheck]
 * @property {string} [commandAck] - with `resultCmdAck`
 * @property {string} [resultCmdAck]
 * @property {string} [uav_state] - with `mission_state`, `wp_reached`, `landed_state`, `protocol`
 * @property {string} [mission_state]
 * @property {string} [wp_reached]
 * @property {number|string} [landed_state] - decoded through `protocol`; alone it sets uav_state too
 * @property {string} [protocol] - 'dji' | 'catec' | anything else = px4 table
 * @property {ArrayLike<number>} [sensors_humidity] - six channels, see HUMIDITY_CHANNELS
 * @property {number|boolean} [threat] - 2 = threat, 3 = confirmed, anything else clears
 */

const LANDED_STATE_PX4 = ['UNDEFINED', 'ON GROUND', 'IN AIR', 'TAKEOFF', 'LANDING'];
const LANDED_STATE_DJI = ['STOPED', 'ON GROUND', 'IN AIR'];

export function convertLandedState(protocol, landedState) {
  if (protocol == 'dji') return LANDED_STATE_DJI[landedState];
  if (protocol == 'catec') return landedState;
  return LANDED_STATE_PX4[landedState];
}

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
      landed_state: 'Ready',
      alarm: 'UNDEFINED',
    },
  };
}

const has = (p, key) => Object.hasOwn(p, key);

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
    apply: (pos, p) => (pos.attributes.gimbal = [round(p.gimbal.x, 1), round(p.gimbal.y, 1), round(p.gimbal.z, 1)]),
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
  {
    when: (p) => has(p, 'localposition'),
    apply: (pos, p) => {
      pos.attributes.localposition = [round(p.localposition.x, 2), round(p.localposition.y, 2), round(p.localposition.z, 2)];
      pos.deviceTime = new Date().toISOString();
    },
  },
  {
    when: (p) => has(p, 'armState'),
    apply: (pos, p) => {
      pos.attributes.armState = p.armState;
      pos.attributes.navState = p.navState;
      pos.attributes.failsafe = p.failsafe;
      pos.attributes.flightCheck = p.flightCheck;
    },
  },
  {
    when: (p) => has(p, 'commandAck'),
    apply: (pos, p) => {
      pos.attributes.commandAck = p.commandAck;
      pos.attributes.resultCmdAck = p.resultCmdAck;
    },
  },
  {
    // uav_state and landed_state are ONE unit: with uav_state it rewrites the mission
    // block; landed_state alone also feeds uav_state.
    when: (p) => has(p, 'uav_state') || has(p, 'landed_state'),
    apply: (pos, p) => {
      const a = pos.attributes;
      const landed = convertLandedState(p.protocol, p.landed_state);
      if (has(p, 'uav_state')) {
        a.mission_state = p.mission_state;
        a.wp_reached = p.wp_reached;
        a.uav_state = p.uav_state;
        // A message without landed_state must not wipe the one we already have.
        if (has(p, 'landed_state')) a.landed_state = landed;
      } else {
        a.uav_state = landed;
        a.landed_state = landed;
      }
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
 * Merges `patch` into `position` (in place) and returns the events to emit.
 * @param {object} position
 * @param {TelemetryPatch} patch
 * @returns {object[]} events for `eventsController.addEvent`
 */
export function applyPatch(position, patch) {
  const effects = [];
  for (const reducer of REDUCERS) {
    if (reducer.when(patch)) reducer.apply(position, patch, effects);
  }
  return effects;
}
