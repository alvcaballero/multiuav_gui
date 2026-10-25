import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { ARM_STATE, FLIGHT_PHASE, FLIGHT_MODE, VTOL_STATE, COMMAND_RESULT } from '../config/status.js';
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
