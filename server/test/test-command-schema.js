import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { readDataFile } from '../common/utils.js';
import { devicesMsg } from '../config/config.js';
import { CommandType } from '../config/commandTypes.js';
import {
  commandSchema,
  validateCommandSchema,
  validateCommandOverrides,
  resolveCommand,
  validateAttributes,
} from '../models/commands/commandSchema.js';

const TYPES = Object.values(CommandType);

const SCHEMA = {
  commands: {
    Takeoff: {
      label: 'Take off',
      group: 'flight',
      params: { altitude: { type: 'number', unit: 'm', min: 1, max: 100, default: 10 } },
    },
    SetupCamera: {
      label: 'Set up camera',
      group: 'camera',
      params: {
        cameraType: {
          type: 'select',
          default: 1,
          options: [
            { value: 0, name: 'FPV' },
            { value: 1, name: 'Main' },
          ],
        },
      },
    },
  },
};
const overrides = (commands) => ({ some_drone: { commands } });

describe('command_schema.yaml (real file)', () => {
  test('has exactly one well-formed entry per CommandType', () => {
    assert.deepEqual(validateCommandSchema(commandSchema, TYPES), []);
  });

  test('the commands: blocks of devices_msg.yaml only narrow the domain', () => {
    assert.deepEqual(validateCommandOverrides(commandSchema, readDataFile(devicesMsg)), []);
  });
});

describe('validateCommandSchema', () => {
  test('reports a CommandType without entry and an entry that is not a CommandType', () => {
    const problems = validateCommandSchema(SCHEMA, ['Takeoff', 'Land']);
    assert.ok(problems.some((p) => p.startsWith('Land: no entry')));
    assert.ok(problems.some((p) => p.startsWith('SetupCamera: not a CommandType')));
  });

  test('reports a default out of range and a select default outside its options', () => {
    const bad = {
      commands: {
        Takeoff: { label: 'x', group: 'flight', params: { altitude: { type: 'number', max: 5, default: 10 } } },
        SetupCamera: { label: 'x', group: 'camera', params: { c: { type: 'select', default: 9, options: [{ value: 0 }] } } },
      },
    };
    const problems = validateCommandSchema(bad, ['Takeoff', 'SetupCamera']);
    assert.ok(problems.some((p) => p.includes('Takeoff.altitude: default 10 out of')));
    assert.ok(problems.some((p) => p.includes('SetupCamera.c: default 9 is not one of its options')));
  });
});

describe('validateCommandOverrides', () => {
  test('a narrowing override is valid', () => {
    const devices = overrides({ Takeoff: { params: { altitude: { min: 2, max: 50 } } } });
    assert.deepEqual(validateCommandOverrides(SCHEMA, devices), []);
  });

  test('an override cannot widen the domain', () => {
    const devices = overrides({ Takeoff: { params: { altitude: { max: 500 } } } });
    assert.ok(validateCommandOverrides(SCHEMA, devices).some((p) => p.includes('max widens 100')));
  });

  test('an override cannot add options nor touch non-overridable fields', () => {
    const devices = overrides({
      SetupCamera: { params: { cameraType: { options: [{ value: 7 }], unit: 'm' } } },
    });
    const problems = validateCommandOverrides(SCHEMA, devices);
    assert.ok(problems.some((p) => p.includes("options add values")));
    assert.ok(problems.some((p) => p.includes("'unit' can't be overridden")));
  });

  test('reports unknown commands and params', () => {
    const devices = overrides({ Fly: {}, Takeoff: { params: { height: { max: 5 } } } });
    const problems = validateCommandOverrides(SCHEMA, devices);
    assert.ok(problems.some((p) => p.includes('commands.Fly: unknown command')));
    assert.ok(problems.some((p) => p.includes('Takeoff.height: unknown param')));
  });
});

describe('validateAttributes', () => {
  const takeoff = resolveCommand(SCHEMA, 'Takeoff');
  const camera = resolveCommand(SCHEMA, 'SetupCamera');

  test('coerces form strings and fills defaults', () => {
    assert.deepEqual(validateAttributes(takeoff, { altitude: '20' }), { ok: true, values: { altitude: 20 } });
    assert.deepEqual(validateAttributes(takeoff, { altitude: '' }), { ok: true, values: { altitude: 10 } });
    assert.deepEqual(validateAttributes(takeoff, undefined), { ok: true, values: { altitude: 10 } });
    assert.deepEqual(validateAttributes(camera, { cameraType: '0' }), { ok: true, values: { cameraType: 0 } });
  });

  test('rejects instead of clamping, naming every failing param', () => {
    const out = validateAttributes(takeoff, { altitude: 500, altitud: 5 });
    assert.equal(out.ok, false);
    assert.match(out.msg, /^Takeoff: /);
    assert.match(out.msg, /'altitud' is not a parameter/);
    assert.match(out.msg, /'altitude' must be <= 100, got 500/);
  });

  test('rejects NaN-producing values, booleans as numbers, and options outside the select', () => {
    assert.match(validateAttributes(takeoff, { altitude: 'abc' }).msg, /'altitude' must be a number/);
    assert.match(validateAttributes(takeoff, { altitude: true }).msg, /'altitude' must be a number/);
    assert.match(validateAttributes(camera, { cameraType: 5 }).msg, /must be one of 0, 1/);
  });

  test('rejects a missing required param and a non-object payload', () => {
    const cmd = { type: 'X', params: [{ id: 'x', type: 'number', required: true }] };
    assert.match(validateAttributes(cmd, {}).msg, /'x' is required/);
    assert.match(validateAttributes(cmd, [1, 2]).msg, /attributes must be an object/);
  });

  test('a param without default that does not come stays absent (handler decides)', () => {
    const gimbal = resolveCommand(commandSchema, 'Gimbal');
    assert.deepEqual(validateAttributes(gimbal, { pitch: -30 }), { ok: true, values: { pitch: -30 } });
  });

  test('a command without params rejects any attribute', () => {
    assert.match(validateAttributes(resolveCommand(commandSchema, 'Land'), { force: 1 }).msg, /'force' is not a parameter/);
  });
});

describe('resolveCommand', () => {
  test('applies the category override on top of the domain', () => {
    const category = { commands: { Takeoff: { params: { altitude: { max: 30, unit: 'ft' } } } } };
    const cmd = resolveCommand(SCHEMA, 'Takeoff', category);
    const [altitude] = cmd.params;
    assert.equal(altitude.id, 'altitude');
    assert.equal(altitude.max, 30);
    assert.equal(altitude.min, 1);
    assert.equal(altitude.unit, 'm'); // unit is domain, not overridable
    assert.equal(altitude.default, 10);
  });

  test('fills presentation defaults', () => {
    const cmd = resolveCommand(SCHEMA, 'Takeoff');
    assert.equal(cmd.severity, 'normal');
    assert.equal(cmd.confirm, null);
    assert.equal(cmd.hidden, false);
  });

  test('an unknown type resolves to undefined', () => {
    assert.equal(resolveCommand(SCHEMA, 'Fly'), undefined);
  });

  test('the real schema resolves every CommandType', () => {
    for (const type of TYPES) assert.equal(resolveCommand(commandSchema, type)?.type, type);
  });
});
