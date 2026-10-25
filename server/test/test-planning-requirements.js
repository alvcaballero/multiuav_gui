import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { readDataFile } from '../common/utils.js';
import { requirementMismatch, requirementProblems, requirementsOf } from '../models/planningRequirements.js';
import { deviceModelsModel } from '../models/deviceModels/deviceModels.js';

// What a mission type of config/planning/config.yaml asks of a device: vehicle types + capabilities,
// never category names (VEHICLE_PLATFORMS_PLAN.md › F6).

const capabilityNames = Object.keys(deviceModelsModel.getCapabilities());
const profile = (deviceModel, category, capabilities) =>
  deviceModelsModel.planningProfileOf({ deviceModel, category, attributes: capabilities ? { capabilities } : {} });

describe('the real config/planning/config.yaml', () => {
  const config = readDataFile('../config/planning/config.yaml');

  test('names only vehicle types and capabilities that exist, and no category list', () => {
    assert.deepEqual(requirementProblems(config, capabilityNames), []);
  });

  test('a tower inspection takes an M300, not an x500 without its optional camera, nor a VTOL', () => {
    const tower = requirementsOf(config.missionTypes.find((m) => m.id === 1).data);
    assert.equal(requirementMismatch(tower, profile('dji_M300', 'dji_M300')), null);
    assert.equal(requirementMismatch(tower, profile('x500', 'px4_ros2')), 'it lacks camera');
    assert.equal(requirementMismatch(tower, profile('x500', 'px4_ros2', { camera: true })), null);
    assert.equal(
      requirementMismatch(tower, profile('standard_vtol', 'px4_mavlink', { camera: true })),
      'it is a vtol, this mission type needs multirotor'
    );
  });
});

describe('requirementProblems', () => {
  const config = (devices) => ({ missionTypes: [{ name: 'm', data: { devices } }] });

  test('an unknown vehicle type or capability would filter every device out: reported', () => {
    const unknown = config({ vehicleTypes: ['quadcopter'], capabilities: ['lidar'] });
    const problems = requirementProblems(unknown, capabilityNames);
    assert.deepEqual(problems, ["m: unknown vehicle type 'quadcopter'", "m: unknown capability 'lidar'"]);
  });

  test('a leftover category list is reported, it is not read any more', () => {
    assert.deepEqual(requirementProblems(config({ category: { type: ['dji_M300'] } }), capabilityNames), [
      "m: 'category' is not read any more (use vehicleTypes / capabilities)",
    ]);
  });
});

describe('requirementMismatch', () => {
  const needs = { vehicleTypes: ['multirotor'], capabilities: ['autonomous_mission'] };

  test('no requirement: any device', () => {
    assert.equal(requirementMismatch(requirementsOf({ devices: { id: {} } }), profile('dji_M600', 'dji_M600')), null);
  });

  test('a missing capability is named', () => {
    assert.equal(requirementMismatch(needs, profile('dji_M600', 'dji_M600')), 'it lacks autonomous_mission');
  });

  test('like the rest of the gating: an unknown model is not filtered by type, an unknown category by nothing', () => {
    assert.equal(requirementMismatch(needs, { vehicleType: null, capabilities: ['autonomous_mission'] }), null);
    assert.equal(requirementMismatch(needs, { vehicleType: 'multirotor', capabilities: null }), null);
  });
});
