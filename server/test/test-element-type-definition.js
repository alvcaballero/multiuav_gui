import { test, describe, after } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'fs';
import path from 'path';
import { elementTypesModel, resolveEffectiveParameterDefs } from '../models/markers/elementTypes.js';

const TEST_TYPE_ID = 'test_definition_yaml_type';

after(async () => {
  const dir = path.join(await elementTypesModel.ensureAssetDir(TEST_TYPE_ID), '..');
  fs.rmSync(path.join(dir, TEST_TYPE_ID), { recursive: true, force: true });
});

describe('resolveEffectiveParameterDefs', () => {
  test('returns the DB copy when present, ignoring any file', () => {
    const dbDefs = [{ key: 'operational_status', label: 'Status', dataType: 'enum', options: ['parked', 'running'] }];
    const result = resolveEffectiveParameterDefs({ id: TEST_TYPE_ID, attributes: { parameterDefs: dbDefs } });
    assert.deepEqual(result, dbDefs);
  });

  test('returns undefined when there is no DB copy and no definition file', () => {
    const result = resolveEffectiveParameterDefs({ id: 'no_such_type_at_all', attributes: null });
    assert.equal(result, undefined);
  });

  test('derives parameterDefs from state_defaults in a stored .type.yaml when DB is empty', async () => {
    const dir = await elementTypesModel.ensureAssetDir(TEST_TYPE_ID);
    fs.writeFileSync(
      path.join(dir, 'definition.yaml'),
      ['format: wtsem-type/0.2', 'state_defaults:', '  nacelle_heading_deg: 240', '  operational_status: parked'].join(
        '\n'
      )
    );

    const result = resolveEffectiveParameterDefs({ id: TEST_TYPE_ID, attributes: null });
    assert.deepEqual(result, [
      { key: 'nacelle_heading_deg', label: 'Nacelle heading deg', dataType: 'number', default: 240 },
      { key: 'operational_status', label: 'Operational status', dataType: 'string', default: 'parked' },
    ]);
  });

  test('treats an empty parameterDefs array in the DB the same as null — falls back to the file', () => {
    // Relies on the good definition.yaml the previous test just wrote.
    const result = resolveEffectiveParameterDefs({ id: TEST_TYPE_ID, attributes: { parameterDefs: [] } });
    assert.deepEqual(result, [
      { key: 'nacelle_heading_deg', label: 'Nacelle heading deg', dataType: 'number', default: 240 },
      { key: 'operational_status', label: 'Operational status', dataType: 'string', default: 'parked' },
    ]);
  });

  test('derives parameterDefs from insem/0.2 `state` — structured entries mixed with bare scalars', async () => {
    const dir = await elementTypesModel.ensureAssetDir(TEST_TYPE_ID);
    fs.writeFileSync(
      path.join(dir, 'definition.yaml'),
      [
        'format: insem/0.2',
        'state:',
        '  upper_trolley_x:',
        '    value: -20',
        '    unit: " m"',
        '    limits: [-52, 52]',
        '    description: Posición del carro superior',
        '  operational_status: parked',
      ].join('\n')
    );

    const result = resolveEffectiveParameterDefs({ id: TEST_TYPE_ID, attributes: null });
    assert.deepEqual(result, [
      {
        key: 'upper_trolley_x',
        label: 'Upper trolley x',
        dataType: 'number',
        default: -20,
        unit: 'm',
        description: 'Posición del carro superior',
        min: -52,
        max: 52,
      },
      { key: 'operational_status', label: 'Operational status', dataType: 'string', default: 'parked' },
    ]);
  });

  test('a malformed definition file yields undefined instead of throwing', async () => {
    const dir = await elementTypesModel.ensureAssetDir(TEST_TYPE_ID);
    fs.writeFileSync(path.join(dir, 'definition.yaml'), '::: not valid yaml :::\n\tbad indent');

    assert.doesNotThrow(() => {
      const result = resolveEffectiveParameterDefs({ id: TEST_TYPE_ID, attributes: null });
      assert.equal(result, undefined);
    });
  });
});
