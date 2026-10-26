// What a mission type of config/planning/config.yaml asks of a device (`data.devices`): the vehicle
// types that can fly it and the capabilities it needs. It replaced a list of category NAMES, which
// named interfaces (how a device is talked to), not what can do the job
// (docs/decisions/vehicle-platforms.md › F6).
// Pure: decodeMissionRequest filters with it, the boot checks the config with it.
import { VEHICLE_TYPE } from '../config/status.js';

/** @returns {{ vehicleTypes: string[], capabilities: string[] }} [] = no requirement */
export function requirementsOf(missionTypeData) {
  const devices = missionTypeData?.devices ?? {};
  return { vehicleTypes: devices.vehicleTypes ?? [], capabilities: devices.capabilities ?? [] };
}

/**
 * What is wrong in the mission types' requirements: an unknown vehicle type or capability would filter
 * every device out with no error, and a leftover `category` list is no longer read.
 * @param {string[]} capabilityNames the capability catalog's keys
 * @returns {string[]}
 */
export function requirementProblems(config, capabilityNames) {
  const problems = [];
  const vehicleTypes = Object.values(VEHICLE_TYPE);
  for (const missionType of config?.missionTypes ?? []) {
    const devices = missionType?.data?.devices ?? {};
    const { vehicleTypes: types, capabilities } = requirementsOf(missionType?.data);
    for (const type of types) {
      if (!vehicleTypes.includes(type)) problems.push(`${missionType.name}: unknown vehicle type '${type}'`);
    }
    for (const name of capabilities) {
      if (!capabilityNames.includes(name)) problems.push(`${missionType.name}: unknown capability '${name}'`);
    }
    if (devices.category) {
      problems.push(`${missionType.name}: 'category' is not read any more (use vehicleTypes / capabilities)`);
    }
  }
  return problems;
}

/**
 * Why a device cannot fly a mission type, or null. Same rule as the rest of the gating: an unknown
 * vehicle type (unknown model) is not filtered by type, unknown capabilities (unknown category) by
 * none.
 * @param {{ vehicleType: string|null, capabilities: string[]|null }} profile deviceModelsModel.planningProfileOf
 */
export function requirementMismatch(requirements, profile) {
  const { vehicleTypes, capabilities } = requirements;
  if (profile.vehicleType && vehicleTypes.length > 0 && !vehicleTypes.includes(profile.vehicleType)) {
    return `it is a ${profile.vehicleType}, this mission type needs ${vehicleTypes.join(' or ')}`;
  }
  const missing = profile.capabilities ? capabilities.filter((name) => !profile.capabilities.includes(name)) : [];
  if (missing.length > 0) return `it lacks ${missing.join(', ')}`;
  return null;
}
