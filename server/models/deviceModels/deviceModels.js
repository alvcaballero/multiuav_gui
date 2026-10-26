// Device models at runtime: what each vehicle is (docs/decisions/vehicle-platforms.md › F2). A device
// is a model + one of the interfaces that model lists (its `category`); this module says which pairs work.
import { readFileSync } from 'node:fs';
import { parse } from 'yaml';
import { capabilitiesFile, deviceModelsDir } from '../../config/config.js';
import { logger } from '../../common/logger.js';
import { categoryModel } from '../category.js';
import {
  buildCapabilityCatalog,
  capabilityOfCommand,
  capabilityOfMissionItem,
  capabilityOfSlot,
  effectiveCapabilities,
} from '../capabilities/capabilityCatalog.js';
import {
  defaultModelOf,
  genericModelOf,
  incompatibility,
  loadDeviceModels,
  modelInterfaceProblem,
} from './deviceModelCatalog.js';
import { formatProblem } from '../interfaces/interfaceCatalog.js';

function reportProblems(problems) {
  if (problems.length === 0) return;
  for (const p of problems) logger.error(`device model ${formatProblem(p)}`);
  if (process.env.LOG_LEVEL === 'debug') throw new Error(`device models: ${problems.map(formatProblem).join('; ')}`);
}

// Every interface a model lists must exist and be one that can drive it: the list is written by hand,
// the compatibility rule (vehicle type + firmware) checks it. A model with a wrong entry is not
// loaded, like an invalid file: its devices are reported at boot instead of offered a dead interface.
function listedInterfaceProblems(catalog) {
  const problems = [];
  const interfaces = categoryModel.getCatalog();
  for (const [modelKey, model] of Object.entries(catalog)) {
    for (const interfaceKey of model.interfaces) {
      const iface = interfaces[interfaceKey];
      const why = iface
        ? incompatibility(modelKey, model, interfaceKey, iface)
        : `lists interface '${interfaceKey}', which does not exist (or was not loaded)`;
      if (why) problems.push({ path: [modelKey, 'interfaces', interfaceKey], message: why });
    }
  }
  return problems;
}

const capabilitiesLoaded = buildCapabilityCatalog(
  parse(readFileSync(capabilitiesFile, 'utf8')),
  categoryModel.missionKeys()
);
const capabilities = capabilitiesLoaded.catalog;

// A model may only name capabilities of the catalog: a typo would silently take the capability away.
function capabilityNameProblems(catalog) {
  const problems = [];
  for (const [modelKey, model] of Object.entries(catalog)) {
    if (model.capabilities === '*') continue;
    for (const name of Object.keys(model.capabilities)) {
      if (!capabilities[name]) problems.push({ path: [modelKey, 'capabilities', name], message: `unknown capability '${name}'` });
    }
  }
  return problems;
}

const loaded = loadDeviceModels(deviceModelsDir);
const listProblems = [...listedInterfaceProblems(loaded.catalog), ...capabilityNameProblems(loaded.catalog)];
const models = Object.fromEntries(
  Object.entries(loaded.catalog).filter(([key]) => !listProblems.some((p) => p.path[0] === key))
);

// An interface's `defaultModel` (what a device created without a model gets, e.g. by the MCP) must
// exist and list that interface.
function defaultModelProblems() {
  const problems = [];
  for (const [key, iface] of Object.entries(categoryModel.getCatalog())) {
    const modelKey = defaultModelOf(iface);
    const model = models[modelKey];
    const why = model ? modelInterfaceProblem(modelKey, model, key, iface) : `unknown device model '${modelKey}'`;
    if (why) problems.push({ path: [key, 'defaultModel'], message: why });
  }
  return problems;
}

// The blocks of a model a planner plans with (Smart-Flies' field names inside each one).
const PLANNING_BLOCKS = ['speeds', 'limits', 'vtol', 'physics', 'battery', 'obstacle_sensor'];

reportProblems(capabilitiesLoaded.problems.map((p) => ({ ...p, path: ['capabilities.yaml', ...p.path] })));
reportProblems([...loaded.problems, ...listProblems]);
reportProblems(defaultModelProblems());

export class deviceModelsModel {
  /** Every model, key → resolved definition. */
  static getAll() {
    return models;
  }

  static get(key) {
    return models[key];
  }

  /**
   * Every model, grouped by vehicle type; with `category`, only the ones that list it, its default
   * first (what an API client choosing for an existing interface needs).
   * @returns {{ name: string, isDefault: boolean, ...model }[]}
   */
  static list({ category } = {}) {
    const defaultKey = category ? this.defaultFor(category) : null;
    return Object.entries(models)
      .filter(([, model]) => !category || model.interfaces.includes(category))
      .map(([key, model]) => ({ name: key, isDefault: key === defaultKey, ...model }))
      .sort(
        (a, b) =>
          b.isDefault - a.isDefault ||
          a.vehicleType.localeCompare(b.vehicleType) ||
          a.name.startsWith('generic_') - b.name.startsWith('generic_') ||
          a.name.localeCompare(b.name)
      );
  }

  /** The interfaces a device of this model can use (its `interfaces`), or [] for an unknown model. */
  static interfacesOf(deviceModel) {
    return models[deviceModel]?.interfaces ?? [];
  }

  /** The model a device of `category` gets when none is given (null for an unknown category). */
  static defaultFor(category) {
    const iface = categoryModel.getCategory(category);
    return iface ? defaultModelOf(iface) : null;
  }

  /**
   * Why a device of this model cannot use this category, or null. Same shape as
   * categoryModel.deviceProblem, which is checked first (an unknown category is its problem).
   * @returns {{path: string[], message: string}|null}
   */
  static deviceProblem({ category, deviceModel }) {
    const iface = categoryModel.getCategory(category);
    if (!iface) return null;
    const model = models[deviceModel];
    if (!model) return { path: ['deviceModel'], message: `unknown device model '${deviceModel}'` };
    const why = modelInterfaceProblem(deviceModel, model, category, iface);
    return why ? { path: ['category'], message: why } : null;
  }

  /** The capability catalog (config/devices/capabilities.yaml), key → { label, description, commands, slots }. */
  static getCapabilities() {
    return capabilities;
  }

  /**
   * What THIS device can do: its model's capabilities (+ its own overrides,
   * `attributes.capabilities`) that its interface implements. A device with an unknown model keeps
   * everything its interface offers, as before device models existed.
   * @param {{ category: string, deviceModel?: string, attributes?: object }} device
   * @returns {Set<string>|null} null when its category is unknown: nothing to gate against (the boot
   *   already reports that device; gating it would only hide what little it has)
   */
  static capabilitiesOf(device) {
    const iface = categoryModel.getCategory(device?.category);
    if (!iface) return null;
    const overrides = device.attributes?.capabilities ?? {};
    const missionItems = categoryModel.missionItemsOf(device.category);
    return effectiveCapabilities(capabilities, models[device.deviceModel], iface, overrides, missionItems);
  }

  /** Whether `device` may carry a mission item (`kind` of MISSION_ITEM_KINDS) its capabilities own. */
  static allowsMissionItem(device, kind, item) {
    const capability = capabilityOfMissionItem(capabilities, kind, item);
    return !capability || (this.capabilitiesOf(device)?.has(capability) ?? true);
  }

  /**
   * What narrows the mission profile of its interface to what THIS device can do: the `restrict` that
   * categoryModel's mission getters take. Equipment (gimbal, camera) through its capabilities, physics
   * through its model's vehicle type (mission_schema.yaml `vehicleTypes`). An unknown model gives no
   * vehicle type: nothing is filtered by type, as nothing is gated for an unknown category.
   */
  static missionRestrictionOf(device) {
    return {
      vehicleType: models[device?.deviceModel]?.vehicleType ?? null,
      allows: (kind, item) => this.allowsMissionItem(device, kind, item),
    };
  }

  /** Whether `device` may be sent `command`: every device has the commands no capability owns. */
  static allowsCommand(device, command) {
    const capability = capabilityOfCommand(capabilities, command);
    return !capability || (this.capabilitiesOf(device)?.has(capability) ?? true);
  }

  /** Whether `device` gets telemetry slot `slot`: subscribed, expected by the watchdog, decoded. */
  static allowsSlot(device, slot) {
    const capability = capabilityOfSlot(capabilities, slot);
    return !capability || (this.capabilitiesOf(device)?.has(capability) ?? true);
  }

  /**
   * Why `overrides` (a device's `attributes.capabilities`) cannot apply to its model, or null: only
   * the capabilities the model declares can be switched (a device never gains one its model lacks),
   * and a `'*'` model has nothing to switch.
   */
  static capabilityOverrideProblem({ deviceModel, attributes }) {
    const overrides = attributes?.capabilities;
    if (!overrides || Object.keys(overrides).length === 0) return null;
    const model = models[deviceModel];
    const declared = model && model.capabilities !== '*' ? model.capabilities : {};
    const unknown = Object.keys(overrides).filter((name) => !(name in declared));
    if (unknown.length === 0) return null;
    return {
      path: ['attributes', 'capabilities', unknown[0]],
      message: `model '${deviceModel}' does not declare capability '${unknown[0]}' (it declares: ${Object.keys(declared).join(', ') || 'none'})`,
    };
  }

  /**
   * What a planner is told about THIS device (docs/decisions/vehicle-platforms.md › F6): what it is, what it can
   * do (effective capabilities, as getAll) and its model's numbers. A block the model does not declare
   * is left out, never filled with a default: the planner then uses its own, as it does when it runs
   * without the GCS. An unknown model gives only the capabilities.
   * @param {{ category: string, deviceModel?: string, attributes?: object }} device
   */
  static planningProfileOf(device) {
    const model = models[device?.deviceModel];
    const capabilities = this.capabilitiesOf(device);
    const profile = {
      deviceModel: device?.deviceModel ?? null,
      vehicleType: model?.vehicleType ?? null,
      capabilities: capabilities ? [...capabilities].sort() : null,
    };
    if (!model) return profile;
    profile.label = model.label ?? device.deviceModel;
    if (model.firmware) profile.firmware = model.firmware;
    for (const block of PLANNING_BLOCKS) {
      if (model[block]) profile[block] = model[block];
    }
    return profile;
  }

  /**
   * What the client draws a device with. A model may leave out its icon or 3D model: the generic of
   * its vehicle type fills them, so a new model is drawn as what it is before it has its own.
   */
  static platformOf(deviceModel) {
    const model = models[deviceModel];
    if (!model) return null;
    const generic = models[genericModelOf(model.vehicleType)] ?? {};
    return {
      vehicleType: model.vehicleType,
      label: model.label ?? deviceModel,
      icon: model.icon ?? generic.icon ?? null,
      model3d: model.model3d ?? generic.model3d ?? null,
    };
  }
}
