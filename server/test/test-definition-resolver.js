import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { resolveDefinitionModel, DefinitionResolveError } from '../models/markers/definitionResolver.js';

describe('resolveDefinitionModel — windTurbine (real stored file)', () => {
  test('resolves without error, one entry per link', () => {
    const model = resolveDefinitionModel('windTurbine');
    assert.ok(model);
    assert.equal(model.format, 'insem/0.2');
    const names = model.links.map((l) => l.name).sort();
    assert.deepEqual(names, ['blade_A', 'blade_B', 'blade_C', 'hub', 'nacelle', 'tower']);
  });

  test('tower is a cylinder with numeric, non-NaN dimensions', () => {
    const model = resolveDefinitionModel('windTurbine');
    const tower = model.links.find((l) => l.name === 'tower');
    assert.equal(tower.parent, 'world');
    assert.equal(tower.joint.type, 'fixed');
    const [shell] = tower.geometry;
    assert.equal(shell.type, 'cylinder');
    assert.ok(Number.isFinite(shell.radiusBottom) && shell.radiusBottom > 0);
    assert.ok(Number.isFinite(shell.radiusTop) && shell.radiusTop > 0);
    assert.ok(Number.isFinite(shell.length) && shell.length > 0);
  });

  test('nacelle is revolute around Z with angle derived from nacelle_heading_deg=240', () => {
    const model = resolveDefinitionModel('windTurbine');
    const nacelle = model.links.find((l) => l.name === 'nacelle');
    assert.equal(nacelle.parent, 'tower');
    assert.equal(nacelle.joint.type, 'revolute');
    assert.deepEqual(nacelle.joint.axis, [0, 0, 1]);
    // value = "90 - nacelle_heading_deg" = 90 - 240 = -150 deg
    assert.ok(Math.abs(nacelle.joint.angleRad - (-150 * Math.PI) / 180) < 1e-9);
  });

  test('blade_A/B/C each resolve a swept_box into N box segments', () => {
    const model = resolveDefinitionModel('windTurbine');
    for (const name of ['blade_A', 'blade_B', 'blade_C']) {
      const blade = model.links.find((l) => l.name === name);
      assert.equal(blade.parent, 'hub');
      const [body] = blade.geometry;
      assert.equal(body.type, 'swept_box');
      assert.equal(body.segments.length, 24); // `segments: 24` in the real file
      for (const seg of body.segments) {
        assert.equal(seg.size.length, 3);
        assert.ok(seg.size.every((v) => Number.isFinite(v) && v > 0));
      }
    }
  });
});

describe('resolveDefinitionModel — content override (unsaved preview)', () => {
  const MINIMAL_YAML = [
    'format: insem/0.2',
    'parameters: { h: 10.0 }',
    'state: { yaw: 30 }',
    'links:',
    '  - name: pole',
    '    parent: world',
    '    joint: { type: fixed }',
    '    geometry: [{ type: cylinder, radius: 0.5, length: h }]',
  ].join('\n');

  test('resolves inline content directly, ignoring the stored file', () => {
    const model = resolveDefinitionModel('windTurbine', MINIMAL_YAML);
    assert.equal(model.links.length, 1);
    assert.equal(model.links[0].name, 'pole');
    assert.equal(model.links[0].geometry[0].length, 10);
  });

  test('a variable used but never defined gives a typed error with the right link', () => {
    const bad = MINIMAL_YAML.replace('radius: 0.5', 'radius: unknown_var');
    assert.throws(
      () => resolveDefinitionModel('windTurbine', bad),
      (err) => {
        assert.ok(err instanceof DefinitionResolveError);
        assert.equal(err.link, 'pole');
        assert.match(err.message, /Undefined variable "unknown_var"/);
        return true;
      }
    );
  });

  test('link resolution does not depend on declaration order', () => {
    const reordered = [
      'format: insem/0.2',
      'parameters: { h: 10.0 }',
      'state: { yaw: 30 }',
      'links:',
      '  - name: child',
      '    parent: pole',
      '    joint: { type: fixed }',
      '    geometry: [{ type: sphere, radius: 1 }]',
      '  - name: pole',
      '    parent: world',
      '    joint: { type: fixed }',
      '    geometry: [{ type: cylinder, radius: 0.5, length: h }]',
    ].join('\n');
    const model = resolveDefinitionModel('windTurbine', reordered);
    assert.equal(model.links.length, 2);
    const child = model.links.find((l) => l.name === 'child');
    assert.equal(child.parent, 'pole');
    assert.equal(child.geometry[0].radius, 1);
  });
});

describe('resolveDefinitionModel — prismatic joint (synthetic, mirrors goliathCrane)', () => {
  const PRISMATIC_YAML = [
    'format: insem/0.2',
    'state: { gantry_position_m: { value: -20, unit: " m" } }',
    'links:',
    '  - name: gantry',
    '    parent: world',
    '    joint: { type: prismatic, axis: [0, 1, 0], value: gantry_position_m }',
    '    geometry: [{ type: box, size: [1, 1, 1] }]',
  ].join('\n');

  test('resolves a prismatic joint into axis + distance from state', () => {
    const model = resolveDefinitionModel('windTurbine', PRISMATIC_YAML);
    const gantry = model.links.find((l) => l.name === 'gantry');
    assert.equal(gantry.joint.type, 'prismatic');
    assert.deepEqual(gantry.joint.axis, [0, 1, 0]);
    assert.equal(gantry.joint.distance, -20);
  });
});

describe('resolveDefinitionModel — goliathCrane (real stored file)', () => {
  test('fails with a clear "unsupported primitive" error — capsule/beam are out of scope this pass', () => {
    // goliathCrane's real file uses `beam` (end_ties, legs) and `capsule`
    // (hoist ropes), neither implemented yet (see resolveGeometryItem).
    // This documents that on purpose, per the "no best-effort, fail loud"
    // design — not a bug in the resolver.
    assert.throws(
      () => resolveDefinitionModel('goliathCrane'),
      (err) => {
        assert.ok(err instanceof DefinitionResolveError);
        assert.match(err.message, /Unsupported primitive type "(beam|capsule)"/);
        return true;
      }
    );
  });
});

describe('resolveDefinitionModel — missing type', () => {
  test('returns null when the type has no stored definition and no content given', () => {
    const model = resolveDefinitionModel('no_such_type_at_all');
    assert.equal(model, null);
  });
});
