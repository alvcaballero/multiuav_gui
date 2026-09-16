import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import {
  GeometrySchema,
  validateElementType,
  validatePartialElementType,
  validateElementItem,
} from '../schemas/zod/markers.js';
import { mergeDefaultGeometry } from '../models/markers/elementItems.js';

describe('GeometrySchema', () => {
  test('accepts a valid circle', () => {
    const result = GeometrySchema.safeParse({
      geometry_type: 'circle',
      dimensions: { radius: 35, height: 80 },
      yaw: 0,
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
});

describe('ElementType/ElementItem attributes validation', () => {
  test('ElementType accepts attributes.geometry alongside free-form keys', () => {
    const result = validateElementType({
      id: 'windTurbine',
      name: 'Wind Turbine',
      attributes: {
        geometry: { geometry_type: 'circle', dimensions: { radius: 35, height: 80 }, yaw: 0 },
        manufacturer: 'Acme',
        bladeCount: 3,
      },
    });
    assert.ok(result.success);
  });

  test('ElementType without attributes still validates (optional)', () => {
    const result = validatePartialElementType({ name: 'Renamed' });
    assert.ok(result.success);
  });

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

  test('ElementItem rejects a malformed nested geometry', () => {
    const result = validateElementItem({
      groupId: 1,
      name: 'Item 1',
      latitude: 1,
      longitude: 2,
      attributes: { geometry: { geometry_type: 'circle', dimensions: { width: 2, length: 3 } } },
    });
    assert.equal(result.success, false);
  });
});

describe('mergeDefaultGeometry', () => {
  const defaultGeometry = { geometry_type: 'circle', dimensions: { radius: 35, height: 80 }, yaw: 0 };

  test('copies the type default when the item has no attributes at all', () => {
    const merged = mergeDefaultGeometry(null, defaultGeometry);
    assert.deepEqual(merged, { geometry: defaultGeometry });
  });

  test('copies the type default into existing attributes without a geometry key', () => {
    const merged = mergeDefaultGeometry({ note: 'inspect quarterly' }, defaultGeometry);
    assert.deepEqual(merged, { note: 'inspect quarterly', geometry: defaultGeometry });
  });

  test('leaves an explicit item-level geometry override untouched', () => {
    const ownGeometry = { geometry_type: 'rectangle', dimensions: { width: 4, length: 8, height: 3 } };
    const merged = mergeDefaultGeometry({ geometry: ownGeometry }, defaultGeometry);
    assert.deepEqual(merged, { geometry: ownGeometry });
  });

  test('returns attributes unchanged when the type has no default geometry', () => {
    const merged = mergeDefaultGeometry({ note: 'no type default' }, undefined);
    assert.deepEqual(merged, { note: 'no type default' });
  });

  test('returns null when there is neither attributes nor a default', () => {
    assert.equal(mergeDefaultGeometry(null, undefined), null);
  });
});
