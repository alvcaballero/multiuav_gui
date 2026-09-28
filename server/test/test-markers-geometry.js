import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import {
  GeometrySchema,
  ParameterDefSchema,
  validateElementType,
  validatePartialElementType,
  validateElementItem,
  validateBase,
} from '../schemas/zod/markers.js';
import { warnUnknownParameterKeys } from '../models/markers/elementItems.js';

describe('GeometrySchema', () => {
  test('accepts a valid circle', () => {
    const result = GeometrySchema.safeParse({
      geometry_type: 'circle',
      dimensions: { radius: 35, height: 80 },
    });
    assert.ok(result.success);
  });

  test('accepts a valid rectangle', () => {
    const result = GeometrySchema.safeParse({
      geometry_type: 'rectangle',
      dimensions: { width: 20, length: 20, height: 2 },
    });
    assert.ok(result.success);
  });

  test('rejects a circle missing radius', () => {
    const result = GeometrySchema.safeParse({
      geometry_type: 'circle',
      dimensions: { height: 10 },
    });
    assert.equal(result.success, false);
  });

  test('rejects a rectangle missing width/length', () => {
    const result = GeometrySchema.safeParse({
      geometry_type: 'rectangle',
      dimensions: { height: 2 },
    });
    assert.equal(result.success, false);
  });

  test('rejects an unknown geometry_type', () => {
    const result = GeometrySchema.safeParse({
      geometry_type: 'triangle',
      dimensions: { radius: 5, height: 5 },
    });
    assert.equal(result.success, false);
  });

  test('rejects non-positive dimensions', () => {
    const result = GeometrySchema.safeParse({
      geometry_type: 'circle',
      dimensions: { radius: 0, height: 5 },
    });
    assert.equal(result.success, false);
  });

  test('no longer accepts yaw — orientation lives on the item as azimFront', () => {
    const result = GeometrySchema.safeParse({
      geometry_type: 'circle',
      dimensions: { radius: 35, height: 80 },
      yaw: 90,
    });
    // `yaw` is simply an unrecognized extra key here — GeometrySchema doesn't
    // fail on unknown keys by default (zod strips them), so the meaningful
    // assertion is that the parsed value never carries it forward.
    assert.ok(result.success);
    assert.equal(result.data.yaw, undefined);
  });
});

describe('ParameterDefSchema', () => {
  test('accepts a number parameter', () => {
    const result = ParameterDefSchema.safeParse({
      key: 'nacelle_heading_deg',
      label: 'Rumbo de la góndola',
      dataType: 'number',
      unit: 'deg',
    });
    assert.ok(result.success);
  });

  test('accepts an enum parameter with options', () => {
    const result = ParameterDefSchema.safeParse({
      key: 'operational_status',
      label: 'Estado operativo',
      dataType: 'enum',
      options: ['parked', 'running', 'maintenance'],
      default: 'parked',
    });
    assert.ok(result.success);
  });

  test('rejects an enum parameter with no options', () => {
    const result = ParameterDefSchema.safeParse({
      key: 'operational_status',
      label: 'Estado operativo',
      dataType: 'enum',
    });
    assert.equal(result.success, false);
  });

  test('rejects a reserved key', () => {
    const result = ParameterDefSchema.safeParse({
      key: 'geometry',
      label: 'Geometry',
      dataType: 'string',
    });
    assert.equal(result.success, false);
  });

  test('rejects an invalid identifier as key', () => {
    const result = ParameterDefSchema.safeParse({
      key: 'nacelle heading',
      label: 'Nacelle heading',
      dataType: 'number',
    });
    assert.equal(result.success, false);
  });
});

describe('ElementType attributes validation', () => {
  test('ElementType accepts attributes.geometry alongside free-form keys', () => {
    const result = validateElementType({
      id: 'windTurbine',
      name: 'Wind Turbine',
      attributes: {
        geometry: { geometry_type: 'circle', dimensions: { radius: 35, height: 80 } },
        manufacturer: 'Acme',
        bladeCount: 3,
      },
    });
    assert.ok(result.success);
  });

  test('ElementType accepts a parameterDefs schema', () => {
    const result = validateElementType({
      id: 'windTurbine',
      name: 'Wind Turbine',
      attributes: {
        parameterDefs: [
          { key: 'nacelle_heading_deg', label: 'Nacelle heading', dataType: 'number', unit: 'deg' },
          {
            key: 'operational_status',
            label: 'Operational status',
            dataType: 'enum',
            options: ['parked', 'running', 'maintenance'],
          },
        ],
      },
    });
    assert.ok(result.success);
  });

  test('ElementType rejects duplicate parameterDefs keys', () => {
    const result = validateElementType({
      id: 'windTurbine',
      name: 'Wind Turbine',
      attributes: {
        parameterDefs: [
          { key: 'blade_pitch_deg', label: 'Blade pitch', dataType: 'number' },
          { key: 'blade_pitch_deg', label: 'Blade pitch (dup)', dataType: 'number' },
        ],
      },
    });
    assert.equal(result.success, false);
  });

  test('ElementType without attributes still validates (optional)', () => {
    const result = validatePartialElementType({ name: 'Renamed' });
    assert.ok(result.success);
  });
});

describe('ElementItem/Base attributes and instance-state validation', () => {
  test('ElementItem still accepts a legacy flat attributes bag (no geometry key)', () => {
    const result = validateElementItem({
      groupId: 1,
      name: 'Item 1',
      latitude: 1,
      longitude: 2,
      attributes: { note: 'legacy', count: 3 },
    });
    assert.ok(result.success);
  });

  test('ElementItem rejects attributes.geometry — dimensions are catalog-only, never a per-item override', () => {
    const result = validateElementItem({
      groupId: 1,
      name: 'Item 1',
      latitude: 1,
      longitude: 2,
      attributes: { geometry: { geometry_type: 'circle', dimensions: { radius: 35, height: 80 } } },
    });
    assert.equal(result.success, false);
  });

  test('ElementItem accepts altitude/azimFront as top-level state', () => {
    const result = validateElementItem({
      groupId: 1,
      name: 'Item 1',
      latitude: 1,
      longitude: 2,
      altitude: 82.5,
      azimFront: 240,
      attributes: { operational_status: 'parked', nacelle_heading_deg: 240 },
    });
    assert.ok(result.success);
  });

  test('ElementItem rejects an out-of-range azimFront', () => {
    const result = validateElementItem({
      groupId: 1,
      name: 'Item 1',
      latitude: 1,
      longitude: 2,
      azimFront: 361,
    });
    assert.equal(result.success, false);
  });

  test('Base now accepts attributes/altitude/azimFront too', () => {
    const result = validateBase({
      typeId: 'windTurbine',
      latitude: 1,
      longitude: 2,
      altitude: 10,
      azimFront: 90,
      attributes: { operational_status: 'running' },
    });
    assert.ok(result.success);
  });

  test('Base rejects attributes.geometry the same way ElementItem does', () => {
    const result = validateBase({
      latitude: 1,
      longitude: 2,
      attributes: { geometry: { geometry_type: 'circle', dimensions: { radius: 1, height: 1 } } },
    });
    assert.equal(result.success, false);
  });
});

describe('warnUnknownParameterKeys', () => {
  test('does not throw for a known key', () => {
    assert.doesNotThrow(() =>
      warnUnknownParameterKeys({ operational_status: 'parked' }, [
        { key: 'operational_status', label: 'Status', dataType: 'enum', options: ['parked', 'running'] },
      ])
    );
  });

  test('does not throw for an unknown key (soft check — logs, never rejects)', () => {
    assert.doesNotThrow(() => warnUnknownParameterKeys({ mystery_field: 1 }, []));
  });

  test('is a no-op when attributes is null/undefined', () => {
    assert.doesNotThrow(() => warnUnknownParameterKeys(null, []));
    assert.doesNotThrow(() => warnUnknownParameterKeys(undefined, undefined));
  });
});
