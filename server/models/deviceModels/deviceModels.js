// Device models at runtime: what each vehicle is (VEHICLE_PLATFORMS_PLAN.md › F2). A device is a
// model + one of the interfaces that model lists (its `category`); this module says which pairs work.
import { deviceModelsDir } from '../../config/config.js';
import { logger } from '../../common/logger.js';
import { categoryModel } from '../category.js';
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

const loaded = loadDeviceModels(deviceModelsDir);
const listProblems = listedInterfaceProblems(loaded.catalog);
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
