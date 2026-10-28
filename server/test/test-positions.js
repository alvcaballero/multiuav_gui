import { test, describe, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { positionsModel } from '../models/positions/positions.js';
import {
  applyPatch,
  createDefaultPosition,
  MISSION_EVENT_REPEAT_MS,
  MISSION_EVENTS_KEPT,
} from '../models/positions/positionReducers.js';
import { ARM_STATE, FLIGHT_MODE, FLIGHT_PHASE, MISSION_EVENT, MISSION_STOP_REASON } from '../config/status.js';
import { eventsController } from '../controllers/events.js';
import { eventBus, EVENTS } from '../common/eventBus.js';

// What positionsModel.updatePosition does with each kind of telemetry patch. Born as
// characterization tests (to prove the reducer extraction changed nothing), then the known
// quirks were fixed and their assertions flipped.
//
// Open question, deliberately NOT pinned as correct: a boolean `threat` (std_msgs/Bool, declared
// by px4_mavros) never raises the alarm because `true == 2` is false. What
// `true` should mean is a hardware decision; see the threat tests below.
//
// Run through `npm test` (it sets DB=false DB_STORAGE=:memory:), never a bare `node --test`.

// Threat events would hit the DB; capture them instead.
const emitted = [];
eventsController.addEvent = (event) => {
  emitted.push(event);
};

// The position cache is module-global: one fresh deviceId per test keeps them independent.
let nextId = 9000;
const newId = () => nextId++;
const update = (deviceId, patch) => positionsModel.updatePosition({ deviceId, ...patch });

beforeEach(() => {
  emitted.length = 0;
});

describe('updatePosition: entry and defaults', () => {
  test('null payload is ignored', async () => {
    assert.equal(await positionsModel.updatePosition(null), null);
  });

  test('first payload creates the default state', async () => {
    const id = newId();
    const pos = await update(id, {});
    assert.deepEqual(pos, {
      deviceId: id,
      accuracy: 0,
      speed: 0,
      course: 0,
      attributes: {
        batteryLevel: 0,
        gimbal: [0, 0, 0],
        obstacle_info: [100, 100, 100, 100, 100, 100],
        takeoff_height: 400,
        mission_state: 'Ready',
        wp_reached: null,
        uav_state: 'OK',
        armState: 'UNKNOWN',
        flightMode: 'UNKNOWN',
        flightPhase: 'UNKNOWN',
        alarm: 'UNDEFINED',
      },
    });
  });

  test('returns the cached object, readable through getByDeviceId', async () => {
    const id = newId();
    const pos = await update(id, { speed: 3 });
    assert.equal(await positionsModel.getByDeviceId(id), pos);
  });

  test('a partial payload keeps every field it does not carry', async () => {
    const id = newId();
    await update(id, { latitude: 37.1, longitude: -5.9, deviceTime: 'T0', altitude: 12, course: 90, speed: 4 });
    const pos = await update(id, { batteryLevel: 55 });
    assert.equal(pos.latitude, 37.1);
    assert.equal(pos.longitude, -5.9);
    assert.equal(pos.altitude, 12);
    assert.equal(pos.course, 90);
    assert.equal(pos.speed, 4);
    assert.equal(pos.attributes.batteryLevel, 55);
  });
});

describe('updatePosition: top-level fields', () => {
  test('latitude carries longitude and deviceTime with it', async () => {
    const pos = await update(newId(), { latitude: 37.1, longitude: -5.9, deviceTime: 'T0' });
    assert.equal(pos.latitude, 37.1);
    assert.equal(pos.longitude, -5.9);
    assert.equal(pos.deviceTime, 'T0');
  });

  test('altitude, course and speed are written independently', async () => {
    const id = newId();
    assert.equal((await update(id, { altitude: 42.5 })).altitude, 42.5);
    assert.equal((await update(id, { course: -90 })).course, -90);
    assert.equal((await update(id, { speed: 7.3 })).speed, 7.3);
  });
});

describe('updatePosition: attributes', () => {
  test('batteryLevel is parsed and rounded to an integer', async () => {
    assert.equal((await update(newId(), { batteryLevel: '87.4' })).attributes.batteryLevel, 87);
    assert.equal((await update(newId(), { batteryLevel: 87.5 })).attributes.batteryLevel, 88);
  });

  test('batteryLevel is NOT scaled here: a 0-1 value rounds to 0 or 1 (scaling is the decoder\'s params.scale)', async () => {
    assert.equal((await update(newId(), { batteryLevel: 0.87 })).attributes.batteryLevel, 1);
    assert.equal((await update(newId(), { batteryLevel: 0.4 })).attributes.batteryLevel, 0);
  });

  test('gimbal becomes a [x, y, z] array rounded to 1 decimal', async () => {
    const pos = await update(newId(), { gimbal: { x: 1.26, y: -2.04, z: 0 } });
    assert.deepEqual(pos.attributes.gimbal, [1.3, -2, 0]);
  });

  test('a gimbal patch without yaw keeps the cached yaw', async () => {
    const id = newId();
    await update(id, { gimbal: { x: 0, y: 0, z: 75 } });
    const pos = await update(id, { gimbal: { x: -30, y: 1 } });
    assert.deepEqual(pos.attributes.gimbal, [-30, 1, 75]);
  });

  test('obstacle_info becomes [down, front, right, back, left, up] rounded to 1 decimal', async () => {
    const pos = await update(newId(), {
      obstacle_info: { down: 1.04, front: 2.06, right: 3, back: 4.44, left: 5.57, up: 6 },
    });
    assert.deepEqual(pos.attributes.obstacle_info, [1, 2.1, 3, 4.4, 5.6, 6]);
  });

  test('obstacle_info with missing sides keeps the previous value of those sides', async () => {
    const id = newId();
    const first = await update(id, { obstacle_info: { down: 1.5 } });
    assert.deepEqual(first.attributes.obstacle_info, [1.5, 100, 100, 100, 100, 100]);
    const second = await update(id, { obstacle_info: { front: 2.04, up: 7 } });
    assert.deepEqual(second.attributes.obstacle_info, [1.5, 2, 100, 100, 100, 7]);
  });

  test('localposition is rounded to 2 decimals and stamps deviceTime with now', async () => {
    const pos = await update(newId(), { localposition: { x: 1.234, y: -5.678, z: 0.004 } });
    assert.deepEqual(pos.attributes.localposition, [1.23, -5.68, 0]);
    assert.ok(Math.abs(Date.now() - Date.parse(pos.deviceTime)) < 2000);
  });

  test('setHome snapshots the current latitude/longitude/altitude', async () => {
    const id = newId();
    await update(id, { latitude: 1, longitude: 2, deviceTime: 'T0' });
    await update(id, { altitude: 3 });
    const pos = await update(id, { setHome: true });
    assert.deepEqual(pos.attributes.home, [1, 2, 3]);
  });

  test('a vehicle_status patch writes arm state, mode, failsafe and flightCheck', async () => {
    const pos = await update(newId(), {
      armState: ARM_STATE.ARMED,
      flightMode: FLIGHT_MODE.MISSION,
      flightModeRaw: 3,
      failsafe: false,
      flightCheck: true,
    });
    assert.equal(pos.attributes.armState, ARM_STATE.ARMED);
    assert.equal(pos.attributes.flightMode, FLIGHT_MODE.MISSION);
    assert.equal(pos.attributes.flightModeRaw, 3);
    assert.equal(pos.attributes.failsafe, false);
    assert.equal(pos.attributes.flightCheck, true);
  });

  test('systemId is kept when a later vehicle_status does not carry it', async () => {
    const id = newId();
    await update(id, { armState: ARM_STATE.ARMED, flightMode: FLIGHT_MODE.MISSION, failsafe: false, flightCheck: true, systemId: 2 });
    const pos = await update(id, { armState: ARM_STATE.DISARMED, flightMode: FLIGHT_MODE.MISSION, failsafe: false, flightCheck: true });
    assert.equal(pos.attributes.systemId, 2);
  });

  test('a source that carries only some of the vehicle state leaves the rest alone', async () => {
    const id = newId();
    await update(id, { armState: ARM_STATE.ARMED, flightMode: FLIGHT_MODE.OFFBOARD, failsafe: true });
    const pos = await update(id, { armState: ARM_STATE.DISARMED, flightPhase: FLIGHT_PHASE.ON_GROUND });
    assert.equal(pos.attributes.armState, ARM_STATE.DISARMED);
    assert.equal(pos.attributes.flightPhase, FLIGHT_PHASE.ON_GROUND);
    assert.equal(pos.attributes.flightMode, FLIGHT_MODE.OFFBOARD);
    assert.equal(pos.attributes.failsafe, true);
  });

  test('commandAck carries resultCmdAck with it', async () => {
    const pos = await update(newId(), { commandAck: 'ARM', resultCmdAck: 'ACCEPTED' });
    assert.equal(pos.attributes.commandAck, 'ARM');
    assert.equal(pos.attributes.resultCmdAck, 'ACCEPTED');
  });
});

describe('updatePosition: mission block and flight phase are independent', () => {
  test('uav_state writes the whole mission block', async () => {
    const pos = await update(newId(), { uav_state: 'ok', mission_state: '1', wp_reached: '3' });
    assert.equal(pos.attributes.mission_state, '1');
    assert.equal(pos.attributes.wp_reached, '3');
    assert.equal(pos.attributes.uav_state, 'ok');
  });

  test('a mission block without flightPhase keeps the phase it already had', async () => {
    const id = newId();
    assert.equal((await update(id, { uav_state: 'ok', mission_state: '0', wp_reached: '0' })).attributes.flightPhase, FLIGHT_PHASE.UNKNOWN);

    await update(id, { flightPhase: FLIGHT_PHASE.IN_AIR });
    const pos = await update(id, { uav_state: 'ok', mission_state: '1', wp_reached: '4' });
    assert.equal(pos.attributes.flightPhase, FLIGHT_PHASE.IN_AIR);
    assert.equal(pos.attributes.mission_state, '1');
    assert.equal(pos.attributes.wp_reached, '4');
  });

  test('flightPhase alone leaves the mission block (and uav_state) alone', async () => {
    const pos = await update(newId(), { flightPhase: FLIGHT_PHASE.IN_AIR, flightPhaseRaw: 2 });
    assert.equal(pos.attributes.flightPhase, FLIGHT_PHASE.IN_AIR);
    assert.equal(pos.attributes.flightPhaseRaw, 2);
    assert.equal(pos.attributes.uav_state, 'OK');
    assert.equal(pos.attributes.mission_state, 'Ready');
    assert.equal(pos.attributes.wp_reached, null);
  });
});

describe('updatePosition: sensors_humidity', () => {
  test('maps the six channels to their attribute names', async () => {
    const pos = await update(newId(), { sensors_humidity: [1, 2, 3, 4, 5, 6] });
    const a = pos.attributes;
    assert.deepEqual([a.MIC_1, a.MIC_2, a.MIC_3, a.Metano, a.Alcohol, a.CO], [1, 2, 3, 4, 5, 6]);
  });

  test('a shorter array only touches the channels it has', async () => {
    const id = newId();
    await update(id, { sensors_humidity: [1, 2, 3, 4, 5, 6] });
    const a = (await update(id, { sensors_humidity: [9] })).attributes;
    assert.deepEqual([a.MIC_1, a.MIC_2, a.CO], [9, 2, 6]);
  });

  test('a reading of 0 is a real value and replaces the previous one', async () => {
    const id = newId();
    await update(id, { sensors_humidity: [7, 8, 9, 1, 1, 1] });
    const a = (await update(id, { sensors_humidity: [0, 0, 0, 0, 0, 0] })).attributes;
    assert.deepEqual([a.MIC_1, a.MIC_2, a.MIC_3, a.Metano, a.Alcohol, a.CO], [0, 0, 0, 0, 0, 0]);
  });
});

describe('updatePosition: threat', () => {
  test('2 raises "threat" and emits one warning; repeating it emits nothing', async () => {
    const id = newId();
    const pos = await update(id, { threat: 2 });
    assert.equal(pos.attributes.alarm, 'threat');
    assert.equal(emitted.length, 1);
    assert.equal(emitted[0].type, 'warning');
    assert.equal(emitted[0].deviceId, id);
    assert.equal(emitted[0].attributes.action, 'Thread');
    assert.equal(emitted[0].attributes.message, 'Detected a threat');

    await update(id, { threat: 2 });
    assert.equal(emitted.length, 1);
  });

  test('3 moves to "confirm" and emits once; any other value clears to "None" silently', async () => {
    const id = newId();
    await update(id, { threat: 2 });
    const confirmed = await update(id, { threat: 3 });
    assert.equal(confirmed.attributes.alarm, 'confirm');
    assert.equal(emitted.length, 2);
    assert.equal(emitted[1].attributes.message, 'Confirmed a threat');

    await update(id, { threat: 3 });
    assert.equal(emitted.length, 2);

    const cleared = await update(id, { threat: 0 });
    assert.equal(cleared.attributes.alarm, 'None');
    assert.equal(emitted.length, 2);

    await update(id, { threat: 2 });
    assert.equal(emitted.length, 3);
  });

  test('OPEN: a boolean threat (std_msgs/Bool) is not 2 or 3, so it never raises the alarm', async () => {
    const pos = await update(newId(), { threat: true });
    assert.equal(pos.attributes.alarm, 'None');
    assert.equal(emitted.length, 0);
  });

  test('the event position is where the vehicle is (stored state), not what the threat message carries', async () => {
    const id = newId();
    await update(id, { latitude: 10, longitude: 20, altitude: 30, deviceTime: 'T0' });
    await update(id, { threat: 2 });
    assert.deepEqual(emitted[0].attributes.positions, { latitude: 10, longitude: 20, altitude: 30 });
  });

  test('the event carries the position when the payload itself has one', async () => {
    await update(newId(), { threat: 2, latitude: 1, longitude: 2, altitude: 3, deviceTime: 'T0' });
    assert.deepEqual(emitted[0].attributes.positions, { latitude: 1, longitude: 2, altitude: 3 });
  });
});

describe('positionReducers: pure module', () => {
  test('applyPatch returns the threat event instead of emitting it', () => {
    const pos = createDefaultPosition(1);
    const effects = applyPatch(pos, { deviceId: 1, threat: 2 });
    assert.equal(effects.length, 1);
    assert.equal(effects[0].attributes.message, 'Detected a threat');
    assert.equal(pos.attributes.alarm, 'threat');
    assert.equal(emitted.length, 0);
  });

  test('createDefaultPosition never shares its arrays between devices', () => {
    const a = createDefaultPosition(1);
    const b = createDefaultPosition(2);
    a.attributes.gimbal[0] = 99;
    a.attributes.obstacle_info[0] = 99;
    assert.deepEqual(b.attributes.gimbal, [0, 0, 0]);
    assert.equal(b.attributes.obstacle_info[0], 100);
  });
});

describe('positionReducers: mission events (DJI WaypointV2)', () => {
  const T0 = 1_000_000;
  const stop = (reason, stamp = 50) => ({ kind: MISSION_EVENT.STOPPED, raw: 3, stamp, reason, reasonRaw: 0 });
  const patch = (missionEvent) => ({ deviceId: 1, missionEvent });

  test('the repeat the wrapper sends is dropped: one timeline entry, one Event', () => {
    const pos = createDefaultPosition(1);
    const first = applyPatch(pos, patch({ kind: MISSION_EVENT.RESUMED, raw: 2, stamp: 7 }), T0);
    const again = applyPatch(pos, patch({ kind: MISSION_EVENT.RESUMED, raw: 2, stamp: 7 }), T0 + 100);
    assert.equal(first.length, 1);
    assert.equal(again.length, 0);
    assert.equal(pos.attributes.missionEvents.length, 1);
  });

  test('the same code and stamp past the window is a new event (a stamp left at 0)', () => {
    const pos = createDefaultPosition(1);
    applyPatch(pos, patch({ kind: MISSION_EVENT.RESUMED, raw: 2, stamp: 0 }), T0);
    applyPatch(pos, patch({ kind: MISSION_EVENT.RESUMED, raw: 2, stamp: 0 }), T0 + MISSION_EVENT_REPEAT_MS + 1);
    assert.equal(pos.attributes.missionEvents.length, 2);
  });

  test('the timeline keeps the newest first, and only the last few', () => {
    const pos = createDefaultPosition(1);
    for (let i = 0; i < MISSION_EVENTS_KEPT + 3; i++) {
      applyPatch(pos, patch({ kind: MISSION_EVENT.WP_REACHED, raw: 16, stamp: i, wp: i }), T0 + i);
    }
    assert.equal(pos.attributes.missionEvents.length, MISSION_EVENTS_KEPT);
    assert.equal(pos.attributes.missionEvents[0].wp, MISSION_EVENTS_KEPT + 2);
  });

  test('a waypoint reached is in the timeline but raises no Event (the task progress shows it)', () => {
    const pos = createDefaultPosition(1);
    assert.deepEqual(applyPatch(pos, patch({ kind: MISSION_EVENT.WP_REACHED, raw: 16, stamp: 1, wp: 0 }), T0), []);
  });

  test('a stop before the end raises a warning AND the signal that closes the task', () => {
    const pos = createDefaultPosition(1);
    const effects = applyPatch(pos, patch(stop(MISSION_STOP_REASON.PREEMPTED)), T0);
    assert.equal(effects[0].type, 'warning');
    assert.match(effects[0].attributes.message, /preempted/);
    assert.deepEqual(effects[1], {
      missionStopped: { deviceId: 1, reason: MISSION_STOP_REASON.PREEMPTED, message: effects[0].attributes.message },
    });
  });

  test('a stop asked from outside is info, still closing the task', () => {
    const effects = applyPatch(createDefaultPosition(1), patch(stop(MISSION_STOP_REASON.USER)), T0);
    assert.equal(effects[0].type, 'info');
    assert.equal(effects[1].missionStopped.reason, MISSION_STOP_REASON.USER);
  });

  test('a stop that is the normal end says nothing: the finish has its own path', () => {
    assert.deepEqual(applyPatch(createDefaultPosition(1), patch(stop(MISSION_STOP_REASON.COMPLETED)), T0), []);
  });

  test('updatePosition sends the stop to the eventBus, not to the Event log', async () => {
    const id = newId();
    const stops = [];
    const onStop = (s) => stops.push(s);
    eventBus.on(EVENTS.DEVICE_MISSION_STOPPED, onStop);
    try {
      await update(id, { missionEvent: stop(MISSION_STOP_REASON.PREEMPTED, 99) });
    } finally {
      eventBus.off(EVENTS.DEVICE_MISSION_STOPPED, onStop);
    }
    assert.equal(stops.length, 1);
    assert.equal(stops[0].deviceId, id);
    assert.equal(emitted.length, 1); // the warning
    assert.equal(emitted[0].attributes.action, 'MissionEvent');
  });
});

describe('positionReducers: control authority and the mission the vehicle runs', () => {
  test('control_auth 0 is written: a false must not be skipped as "absent"', () => {
    const pos = createDefaultPosition(1);
    applyPatch(pos, { deviceId: 1, controlSource: 'ONBOARD', controlAuthority: true, controlSourceRaw: 4 });
    applyPatch(pos, { deviceId: 1, controlSource: 'PILOT', controlAuthority: false, controlSourceRaw: 0 });
    assert.equal(pos.attributes.controlSource, 'PILOT');
    assert.equal(pos.attributes.controlAuthority, false);
  });

  test('the mission state keys are kept', () => {
    const pos = createDefaultPosition(1);
    const state = { missionExecState: 'PAUSED', missionExecStateRaw: 4, missionExecWp: 3, missionExecSpeed: 2.5 };
    applyPatch(pos, { deviceId: 1, ...state });
    assert.equal(pos.attributes.missionExecState, 'PAUSED');
    assert.equal(pos.attributes.missionExecWp, 3);
  });
});
