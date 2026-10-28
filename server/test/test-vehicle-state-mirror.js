import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import {
  ARM_STATE,
  FLIGHT_PHASE,
  FLIGHT_MODE,
  VTOL_STATE,
  COMMAND_RESULT,
  CONTROL_SOURCE,
  MISSION_EXEC_STATE,
  MISSION_EVENT,
  MISSION_STOP_REASON,
} from '../config/status.js';
import * as client from '../../client/src/common/util/vehicleState.js';

// The client keeps its own copy of the canonical vehicle state (it can't import the server).
// A value renamed on one side only would make the UI show nothing for it, silently.
describe('client vehicle state mirror', () => {
  test('ARM_STATE matches the server', () => {
    assert.deepEqual({ ...client.ARM_STATE }, { ...ARM_STATE });
  });

  test('FLIGHT_PHASE matches the server', () => {
    assert.deepEqual({ ...client.FLIGHT_PHASE }, { ...FLIGHT_PHASE });
  });

  test('FLIGHT_MODE matches the server', () => {
    assert.deepEqual({ ...client.FLIGHT_MODE }, { ...FLIGHT_MODE });
  });

  test('VTOL_STATE matches the server', () => {
    assert.deepEqual({ ...client.VTOL_STATE }, { ...VTOL_STATE });
  });

  test('COMMAND_RESULT matches the server', () => {
    assert.deepEqual({ ...client.COMMAND_RESULT }, { ...COMMAND_RESULT });
  });

  test('CONTROL_SOURCE and the MISSION_* vocabularies match the server', () => {
    assert.deepEqual({ ...client.CONTROL_SOURCE }, { ...CONTROL_SOURCE });
    assert.deepEqual({ ...client.MISSION_EXEC_STATE }, { ...MISSION_EXEC_STATE });
    assert.deepEqual({ ...client.MISSION_EVENT }, { ...MISSION_EVENT });
    assert.deepEqual({ ...client.MISSION_STOP_REASON }, { ...MISSION_STOP_REASON });
  });

  test('every control source, mission state and event the vehicle reports has text to show', () => {
    for (const v of Object.values(CONTROL_SOURCE).filter((x) => x !== 'UNKNOWN')) {
      assert.ok(client.controlLabel(v), `control ${v}`);
    }
    for (const v of Object.values(MISSION_EXEC_STATE).filter((x) => x !== 'UNKNOWN')) {
      assert.ok(client.missionExecLabel(v), `mission state ${v}`);
    }
    // OTHER never reaches the client: the decoder drops it.
    for (const kind of Object.values(MISSION_EVENT).filter((x) => x !== 'OTHER')) {
      assert.ok(client.missionEventLabel({ kind, reason: MISSION_STOP_REASON.PREEMPTED, wp: 1 }), `event ${kind}`);
    }
    for (const reason of Object.values(MISSION_STOP_REASON)) {
      assert.doesNotMatch(client.missionEventLabel({ kind: MISSION_EVENT.STOPPED, reason }), /undefined/, reason);
    }
  });

  test('only a stop before the end is abnormal', () => {
    assert.equal(client.isAbnormalStop({ kind: MISSION_EVENT.STOPPED, reason: MISSION_STOP_REASON.PREEMPTED }), true);
    assert.equal(client.isAbnormalStop({ kind: MISSION_EVENT.STOPPED, reason: MISSION_STOP_REASON.COMPLETED }), false);
    assert.equal(client.isAbnormalStop({ kind: MISSION_EVENT.RESUMED }), false);
  });

  test('every command result except UNKNOWN has text to show', () => {
    for (const v of Object.values(COMMAND_RESULT).filter((x) => x !== 'UNKNOWN')) assert.ok(client.resultLabel(v), `result ${v}`);
    assert.equal(client.resultLabel('UNKNOWN'), null);
  });

  test('every value except UNKNOWN has text to show', () => {
    for (const v of Object.values(ARM_STATE).filter((x) => x !== 'UNKNOWN')) assert.ok(client.armLabel(v), `arm ${v}`);
    for (const v of Object.values(FLIGHT_PHASE).filter((x) => x !== 'UNKNOWN')) assert.ok(client.phaseLabel(v), `phase ${v}`);
    for (const v of Object.values(FLIGHT_MODE).filter((x) => x !== 'UNKNOWN')) assert.ok(client.modeLabel(v), `mode ${v}`);
    for (const v of Object.values(VTOL_STATE).filter((x) => x !== 'UNKNOWN')) assert.ok(client.vtolLabel(v), `vtol ${v}`);
  });

  test('UNKNOWN and absent show nothing', () => {
    assert.equal(client.armLabel('UNKNOWN'), null);
    assert.equal(client.phaseLabel(undefined), null);
    assert.equal(client.modeLabel('UNKNOWN'), null);
    assert.equal(client.vtolLabel(undefined), null);
  });
});
