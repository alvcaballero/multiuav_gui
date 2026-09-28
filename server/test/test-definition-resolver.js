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

describe('resolveDefinitionModel — goliathCrane (real stored file, beam/capsule)', () => {
  test('resolves every link, including ones built entirely from beam/capsule', () => {
    const model = resolveDefinitionModel('goliathCrane');
    assert.ok(model);
    const names = model.links.map((l) => l.name).sort();
    assert.ok(names.includes('end_ties'));
    assert.ok(names.includes('rigid_leg'));
    assert.ok(names.includes('upper_hoist'));
    assert.ok(names.includes('lower_hoist'));
  });

  test('end_ties beams resolve position/quaternion/length/size as finite numbers', () => {
    const model = resolveDefinitionModel('goliathCrane');
    const endTies = model.links.find((l) => l.name === 'end_ties');
    for (const beam of endTies.geometry) {
      assert.equal(beam.type, 'beam');
      assert.equal(beam.position.length, 3);
      assert.ok(beam.position.every(Number.isFinite));
      assert.equal(beam.quaternion.length, 4);
      assert.ok(beam.quaternion.every(Number.isFinite));
      const [qx, qy, qz, qw] = beam.quaternion;
      assert.ok(Math.abs(Math.hypot(qx, qy, qz, qw) - 1) < 1e-9); // must stay a unit quaternion
      assert.ok(Number.isFinite(beam.length) && beam.length > 0);
      assert.deepEqual(beam.size, [3.0, 2.5]); // [depth, width] from `width: 2.5, depth: 3.0`
    }
  });

  test('a beam with no explicit depth falls back to width (square section)', () => {
    // rigid_leg's columns only set `width`, matching insem.py's
    // `g.get("depth", g["width"])` default.
    const model = resolveDefinitionModel('goliathCrane');
    const rigidLeg = model.links.find((l) => l.name === 'rigid_leg');
    const col = rigidLeg.geometry.find((g) => g.id === 'col_N');
    assert.deepEqual(col.size, [3.2, 3.2]); // rigid_section = 3.2
  });

  test('upper_hoist/lower_hoist ropes resolve as capsules with the configured radius', () => {
    const model = resolveDefinitionModel('goliathCrane');
    for (const linkName of ['upper_hoist', 'lower_hoist']) {
      const hoist = model.links.find((l) => l.name === linkName);
      const ropes = hoist.geometry.find((g) => g.id === 'ropes');
      assert.equal(ropes.type, 'capsule');
      assert.equal(ropes.radius, 0.8); // rope_radius
      assert.ok(Number.isFinite(ropes.length) && ropes.length > 0);
      assert.deepEqual(ropes.position, [0, 0, 0]); // a = [0, 0, 0]
      const [qx, qy, qz, qw] = ropes.quaternion;
      assert.ok(Math.abs(Math.hypot(qx, qy, qz, qw) - 1) < 1e-9);
    }
  });
});

describe('resolveDefinitionModel — missing type', () => {
  test('returns null when the type has no stored definition and no content given', () => {
    const model = resolveDefinitionModel('no_such_type_at_all');
    assert.equal(model, null);
  });
});
