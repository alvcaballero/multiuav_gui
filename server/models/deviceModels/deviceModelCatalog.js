// The device model catalog: WHAT each vehicle is (vehicle type, firmware, physics, icon), one file per
// model in config/devices/device_models/. Read, validated and resolved with no side effects, like the
// interface catalog (models/interfaces/interfaceCatalog.js), which says HOW the GCS talks to it.
import { VEHICLE_TYPE } from '../../config/status.js';
import { deviceModelShapeProblems } from '../../schemas/zod/deviceModel.js';
import { readInterfaceDir } from '../interfaces/interfaceStore.js';
import { resolveExtends } from '../catalog/resolveExtends.js';

const isPlainObject = (value) => value !== null && typeof value === 'object' && !Array.isArray(value);

/** `child` over its resolved `parent`: objects merge key by key, the rest is replaced. */
export function mergeDeviceModel(parent, child) {
  const out = { ...parent };
  for (const [key, value] of Object.entries(child)) {
    out[key] = isPlainObject(value) && isPlainObject(parent[key]) ? mergeDeviceModel(parent[key], value) : value;
  }
  if (!child.abstract) delete out.abstract;
  return out;
}

/** The model a vehicle type falls back to when nothing more specific is known. */
export const genericModelOf = (vehicleType) => `generic_${vehicleType}`;

const prefixed = (key, problems) => problems.map(({ path, message }) => ({ path: [key, ...path], message }));

/**
 * @returns {{ catalog: Record<string, object>, problems: {path: (string|number)[], message: string}[] }}
 *   `catalog`: the concrete models that validated (abstract bases are only for `extends`)
 */
export function buildDeviceModelCatalog(raw) {
  const problems = [];
  const shaped = {};
  for (const [key, def] of Object.entries(raw)) {
    const own = deviceModelShapeProblems(def);
    if (own.length) problems.push(...prefixed(key, own));
    else shaped[key] = def ?? {};
  }
  const { resolved, problems: extendsProblems } = resolveExtends(shaped, Object.keys(raw), mergeDeviceModel);
  problems.push(...extendsProblems);

  const catalog = {};
  for (const [key, def] of Object.entries(resolved)) {
    if (def.abstract) continue;
    const own = [];
    if (!def.vehicleType) own.push({ path: [key, 'vehicleType'], message: 'missing vehicleType' });
    if (!def.interfaces) own.push({ path: [key, 'interfaces'], message: 'missing interfaces (the list may be empty)' });
    if (own.length) problems.push(...own);
    else catalog[key] = def;
  }
  // Every vehicle type has its generic model: it is what a device gets when nothing else applies.
  for (const vehicleType of Object.values(VEHICLE_TYPE)) {
    const generic = genericModelOf(vehicleType);
    if (catalog[generic]?.vehicleType !== vehicleType) {
      problems.push({ path: [generic], message: `missing the generic model of '${vehicleType}' (vehicleType: ${vehicleType})` });
    }
  }
  return { catalog, problems };
}

export function loadDeviceModels(dir) {
  const { raw, problems: fileProblems } = readInterfaceDir(dir);
  const { catalog, problems } = buildDeviceModelCatalog(raw);
  return { raw, catalog, problems: [...fileProblems, ...problems] };
}

/**
 * Why a device of model `model` cannot be driven by interface `iface`, or null. The interface must
 * serve the model's vehicle type and drive one of the firmwares it runs (a model that declares no
 * firmware is not known, so any firmware of the interface will do).
 * @param {string} modelKey @param {object} model resolved model
 * @param {string} interfaceKey @param {object} iface resolved interface
 */
export function incompatibility(modelKey, model, interfaceKey, iface) {
  if (!iface.vehicleTypes?.includes(model.vehicleType)) {
    return `model '${modelKey}' is a ${model.vehicleType}, and interface '${interfaceKey}' serves ${(iface.vehicleTypes ?? []).join(', ')}`;
  }
  if (model.firmware && !model.firmware.some((fw) => iface.firmwares?.includes(fw))) {
    return `model '${modelKey}' runs ${model.firmware.join(', ')}, and interface '${interfaceKey}' drives ${(iface.firmwares ?? []).join(', ')}`;
  }
  return null;
}

/**
 * Why a device of model `modelKey` cannot use interface `interfaceKey`, or null: the model must list
 * it (`interfaces`), and the pair must pass the compatibility rule (`incompatibility`). The boot checks
 * the second for every listed interface, so on a running server only the first can fail.
 */
export function modelInterfaceProblem(modelKey, model, interfaceKey, iface) {
  if (!model.interfaces?.includes(interfaceKey)) {
    return `model '${modelKey}' does not list interface '${interfaceKey}' (its interfaces: ${(model.interfaces ?? []).join(', ') || 'none'})`;
  }
  return incompatibility(modelKey, model, interfaceKey, iface);
}

/** The model a new device of `interfaceKey` gets: the interface's `defaultModel`, or the generic of its first vehicle type. */
export const defaultModelOf = (iface) => iface.defaultModel ?? genericModelOf(iface.vehicleTypes[0]);
