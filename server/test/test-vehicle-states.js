import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import {
  ARM_STATE,
  FLIGHT_PHASE,
  FLIGHT_MODE,
  COMMAND_RESULT,
  CONTROL_LEVEL,
  controlLevelOf,
  isAutonomous,
} from '../config/status.js';
import { px4ArmState, px4FlightMode, px4FlightPhase, px4CommandResult, djiFlightStatus, djiFlightMode } from '../models/vehicleStates.js';

const values = (o) => Object.values(o);

describe('canonical vehicle state', () => {
  test('every flight mode says who is in control', () => {
    for (const mode of values(FLIGHT_MODE)) {
      assert.ok(values(CONTROL_LEVEL).includes(controlLevelOf(mode)), `${mode} has no control level`);
    }
  });

  test('isAutonomous: the autopilot or an external computer, never the pilot', () => {
    assert.equal(isAutonomous(FLIGHT_MODE.MISSION), true);
    assert.equal(isAutonomous(FLIGHT_MODE.OFFBOARD), true);
    assert.equal(isAutonomous(FLIGHT_MODE.MANUAL), false);
    assert.equal(isAutonomous(FLIGHT_MODE.ASSISTED), false);
    assert.equal(isAutonomous(FLIGHT_MODE.UNKNOWN), false);
    assert.equal(isAutonomous('not-a-mode'), false);
  });

  test('offboard is external, not autopilot-auto', () => {
    assert.equal(controlLevelOf(FLIGHT_MODE.OFFBOARD), CONTROL_LEVEL.EXTERNAL);
    assert.equal(controlLevelOf(FLIGHT_MODE.RTL), CONTROL_LEVEL.AUTO);
  });
});

describe('PX4 adapters', () => {
  test('arming_state', () => {
    assert.equal(px4ArmState(1), ARM_STATE.DISARMED);
    assert.equal(px4ArmState(2), ARM_STATE.ARMED);
    assert.equal(px4ArmState(99), ARM_STATE.UNKNOWN);
  });

  test('nav_state groups the firmware modes into the GCS ones', () => {
    assert.equal(px4FlightMode(0), FLIGHT_MODE.MANUAL);
    assert.equal(px4FlightMode(2), FLIGHT_MODE.ASSISTED);
    assert.equal(px4FlightMode(3), FLIGHT_MODE.MISSION);
    assert.equal(px4FlightMode(5), FLIGHT_MODE.RTL);
    assert.equal(px4FlightMode(12), FLIGHT_MODE.LAND);
    assert.equal(px4FlightMode(13), FLIGHT_MODE.EMERGENCY);
    assert.equal(px4FlightMode(14), FLIGHT_MODE.OFFBOARD);
  });

  test('a nav_state with no mapping is OTHER, never a crash or a firmware label', () => {
    assert.equal(px4FlightMode(7), FLIGHT_MODE.OTHER); // Free5
    assert.equal(px4FlightMode(25), FLIGHT_MODE.OTHER); // External3
    assert.equal(px4FlightMode(undefined), FLIGHT_MODE.OTHER);
  });

  test('every mapped nav_state lands on a canonical mode', () => {
    for (let code = 0; code <= 31; code++) {
      assert.ok(values(FLIGHT_MODE).includes(px4FlightMode(code)), `nav_state ${code}`);
    }
  });

  test('landed_state (MAV_LANDED_STATE)', () => {
    assert.equal(px4FlightPhase(0), FLIGHT_PHASE.UNKNOWN);
    assert.equal(px4FlightPhase(1), FLIGHT_PHASE.ON_GROUND);
    assert.equal(px4FlightPhase(2), FLIGHT_PHASE.IN_AIR);
    assert.equal(px4FlightPhase(3), FLIGHT_PHASE.TAKEOFF);
    assert.equal(px4FlightPhase(4), FLIGHT_PHASE.LANDING);
    assert.equal(px4FlightPhase(9), FLIGHT_PHASE.UNKNOWN);
  });
});

describe('PX4 command result', () => {
  test('MAV_RESULT codes', () => {
    const expected = ['ACCEPTED', 'REJECTED', 'DENIED', 'UNSUPPORTED', 'FAILED', 'IN_PROGRESS', 'CANCELLED'];
    expected.forEach((result, code) => assert.equal(px4CommandResult(code), result, `result ${code}`));
  });

  test('an unknown code is UNKNOWN, never a crash or a firmware label', () => {
    assert.equal(px4CommandResult(99), COMMAND_RESULT.UNKNOWN);
    assert.equal(px4CommandResult(undefined), COMMAND_RESULT.UNKNOWN);
  });

  test('every mapped code lands on a canonical result', () => {
    for (let code = 0; code <= 7; code++) assert.ok(values(COMMAND_RESULT).includes(px4CommandResult(code)), `result ${code}`);
  });
});

describe('DJI adapter', () => {
  test('flight_status carries both axes: STOPED is disarmed on the ground', () => {
    assert.deepEqual(djiFlightStatus(0), { armState: ARM_STATE.DISARMED, flightPhase: FLIGHT_PHASE.ON_GROUND });
    assert.deepEqual(djiFlightStatus(1), { armState: ARM_STATE.ARMED, flightPhase: FLIGHT_PHASE.ON_GROUND });
    assert.deepEqual(djiFlightStatus(2), { armState: ARM_STATE.ARMED, flightPhase: FLIGHT_PHASE.IN_AIR });
  });

  test('an unknown code says nothing instead of guessing', () => {
    assert.deepEqual(djiFlightStatus(6), { armState: ARM_STATE.UNKNOWN, flightPhase: FLIGHT_PHASE.UNKNOWN });
  });

  test('display_mode groups the DJI modes into the GCS ones', () => {
    assert.equal(djiFlightMode(0), FLIGHT_MODE.MANUAL);
    assert.equal(djiFlightMode(1), FLIGHT_MODE.ASSISTED); // attitude
    assert.equal(djiFlightMode(6), FLIGHT_MODE.ASSISTED); // P-GPS
    assert.equal(djiFlightMode(11), FLIGHT_MODE.TAKEOFF);
    assert.equal(djiFlightMode(12), FLIGHT_MODE.LAND);
    assert.equal(djiFlightMode(33), FLIGHT_MODE.LAND); // forced landing
    assert.equal(djiFlightMode(15), FLIGHT_MODE.RTL);
    assert.equal(djiFlightMode(17), FLIGHT_MODE.OFFBOARD);
  });

  test('a reserved or unmapped display_mode is OTHER, never a crash', () => {
    assert.equal(djiFlightMode(2), FLIGHT_MODE.OTHER); // RESERVED_2
    assert.equal(djiFlightMode(41), FLIGHT_MODE.OTHER); // ENGINE_START
    assert.equal(djiFlightMode(undefined), FLIGHT_MODE.OTHER);
  });

  test('every display_mode lands on a canonical mode', () => {
    for (let code = 0; code <= 43; code++) assert.ok(values(FLIGHT_MODE).includes(djiFlightMode(code)), `display_mode ${code}`);
  });
});
