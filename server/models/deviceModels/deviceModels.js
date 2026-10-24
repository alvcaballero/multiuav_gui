// Device models at runtime: what each vehicle is (VEHICLE_PLATFORMS_PLAN.md › F2). A device is a
// model + an interface (its `category`); this module says which pairs work.
import { deviceModelsDir } from '../../config/config.js';
import { logger } from '../../common/logger.js';
import { categoryModel } from '../category.js';
import { defaultModelOf, genericModelOf, incompatibility, loadDeviceModels } from './deviceModelCatalog.js';
import { formatProblem } from '../interfaces/interfaceCatalog.js';

const loaded = loadDeviceModels(deviceModelsDir);
const models = loaded.catalog;

function reportProblems(problems) {
  if (problems.length === 0) return;
  for (const p of problems) logger.error(`device model ${formatProblem(p)}`);
  if (process.env.LOG_LEVEL === 'debug') throw new Error(`device models: ${problems.map(formatProblem).join('; ')}`);
}

// An interface's `defaultModel` must exist and be one it can drive: it is what a device of that
// interface gets when nobody chose a model.
function defaultModelProblems() {
  const problems = [];
  for (const [key, iface] of Object.entries(categoryModel.getCatalog())) {
    const modelKey = defaultModelOf(iface);
    const model = models[modelKey];
    if (!model) problems.push({ path: [key, 'defaultModel'], message: `unknown device model '${modelKey}'` });
    else {
      const why = incompatibility(modelKey, model, key, iface);
      if (why) problems.push({ path: [key, 'defaultModel'], message: why });
    }
  }
  return problems;
}

reportProblems(loaded.problems);
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
   * The models a device of `category` can be, its default first. Without a category: all of them.
   * @returns {{ name: string, isDefault: boolean, ...model }[]}
   */
  static list({ category } = {}) {
    const iface = category ? categoryModel.getCategory(category) : null;
    if (category && !iface) return [];
    const defaultKey = iface ? defaultModelOf(iface) : null;
    return Object.entries(models)
      .filter(([key, model]) => !iface || !incompatibility(key, model, category, iface))
      .map(([key, model]) => ({ name: key, isDefault: key === defaultKey, ...model }))
      .sort((a, b) => b.isDefault - a.isDefault || a.name.localeCompare(b.name));
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
    const why = incompatibility(deviceModel, model, category, iface);
    return why ? { path: ['deviceModel'], message: why } : null;
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
