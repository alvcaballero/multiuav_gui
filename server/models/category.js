import { readDataFile, writeDataFile } from '../common/utils.js';
import { devicesMsg, missionSchema, messagesTypes } from '../config/config.js';
import { KNOWN_SERVICE_KEYS, KNOWN_TOPIC_KEYS } from '../config/deviceMsgCatalog.js';
import logger from '../common/logger.js';
import { symbolsForService, profileFor } from './mission/missionSymbols.js';

const devices_msg = readDataFile(devicesMsg);
const messages_types = readDataFile(messagesTypes);
const _mission_schema = readDataFile(missionSchema);

// Valida que cada key de los bloques topics/services/actions en devices_msg esté
// registrada en su catálogo correspondiente. Una key huérfana significa que un UAV
// declara una capacidad que el sistema no sabe manejar:
//   - topics  → key sin entrada en deviceMsgCatalog (posible typo o topic no soportado)
//   - services/actions → sin entrada en commandCatalog → comando invisible o roto
//
// Cada bloque se valida contra SU SSOT (topics ≠ commands: naturalezas distintas).
// warn por defecto; en modo debug (LOG_LEVEL=debug) lanza para forzar la alineación
// temprano — un typo en el YAML no debe llegar silencioso a runtime.
const DEVICES_MSG_BLOCKS = {
  topics: KNOWN_TOPIC_KEYS,
  services: KNOWN_SERVICE_KEYS,
  actions: KNOWN_SERVICE_KEYS,
};

function validateDevicesMsgKeys(catalog) {
  const strict = process.env.LOG_LEVEL === 'debug';
  const orphans = [];
  for (const [category, def] of Object.entries(catalog)) {
    for (const [block, knownKeys] of Object.entries(DEVICES_MSG_BLOCKS)) {
      for (const key of Object.keys(def?.[block] ?? {})) {
        if (!knownKeys.has(key)) orphans.push(`${category}.${block}.${key}`);
      }
    }
  }
  if (orphans.length === 0) return;
  const msg = `devices_msg: ${orphans.length} unknown key(s) not in catalog: ${orphans.join(', ')}`;
  if (strict) throw new Error(msg);
  logger.warn(msg);
}

validateDevicesMsgKeys(devices_msg);

// ─── Mission catalog resolution (enriched catalog → per-category profile) ──────

// Returns catalog entries for the given keys, preserving order. Unknown keys are
// logged (surfaces a typo'd capability instead of silently dropping it).
function pick(catalog, keys) {
  const out = {};
  for (const key of keys ?? []) {
    if (catalog[key] === undefined) {
      logger.warn(`mission_schema: profile references unknown key '${key}'`);
      continue;
    }
    out[key] = catalog[key];
  }
  return out;
}

// Filters a select param's options to those the encoder family actually supports.
// The family's symbol table (source of truth) decides: an option whose `key` isn't
// mapped is dropped, so the UI never offers a mode the firmware can't execute.
function filterParamOptions(param, symbols) {
  if (param.type !== 'select' || !symbols) return param;
  const supported = symbols[param.id];
  if (!supported) return param; // no table for this param → leave options as-is
  return { ...param, options: param.options.filter((opt) => opt.key in supported) };
}

// Resolves the actions/params a category exposes by filtering the global catalog
// through its profile AND its encoder family's capabilities.
//
// The profile is DERIVED from the configureMission serviceType + category (see
// profileFor), so it can't contradict the encoder. An explicit `mission:` field
// on the device still wins as an escape hatch. No device/service → 'default',
// unfiltered, so the planning UI always has a coherent baseline.
function resolveMissionProfile(type) {
  const dev = devices_msg[type];
  const serviceType = dev?.services?.configureMission?.serviceType;
  const profileName = dev?.mission ?? dev?.mission_schema ?? profileFor(type, serviceType);
  const profile = _mission_schema.profiles?.[profileName] ?? _mission_schema.profiles?.default;
  if (!profile) {
    logger.warn(`mission_schema: no profile resolvable for category '${type}' (and no 'default' profile)`);
    return undefined;
  }

  // Encoder family (via configureMission serviceType) decides supported options.
  // No device/service → symbols is null → options are left unfiltered (catalog full set).
  const symbols = symbolsForService(serviceType);
  const filterAll = (defs) => {
    const out = {};
    for (const [key, p] of Object.entries(defs)) out[key] = filterParamOptions(p, symbols);
    return out;
  };
  return {
    actions: pick(_mission_schema.actions, profile.actions),
    params: filterAll(pick(_mission_schema.params, profile.params)),
    waypointParams: filterAll(pick(_mission_schema.waypoint_params ?? {}, profile.waypoint_params)),
  };
}

// Formats a catalog param def into the wire shape consumed by the UI: legacy
// {id,name,type,default} superset with rich metadata (description/unit/range/options).
function formatParamDef(p) {
  return {
    id: p.id,
    name: p.Name,
    type: p.type,
    default: p.default ?? null,
    description: p.description ?? null,
    unit: p.unit ?? null,
    min: p.min ?? null,
    max: p.max ?? null,
    step: p.step ?? null,
    options: p.options ?? null,
  };
}

// number (wire) → canonical symbol, read from the catalog. First step of the
// number → symbol → firmware-number translation. Looks in both route-level params
// and per-waypoint params (e.g. mode_turn). Returns undefined if unmapped.
function symbolForValue(param, value) {
  const def = _mission_schema.params?.[param] ?? _mission_schema.waypoint_params?.[param];
  return def?.options?.find((opt) => opt.value === value)?.key;
}

export class categoryModel {
  static getAll() {
    logger.debug('categoryModel.getAll');
    return Object.keys(devices_msg);
  }
  static getCategory(type) {
    logger.debug(`categoryModel.getCategory: ${type}`);
    return devices_msg[type];
  }
  static updateCategory(type, value) {
    logger.info(`categoryModel.updateCategory: ${type}`);
    if (devices_msg.hasOwnProperty(type)) {
      devices_msg[type] = value;
      writeDataFile(devicesMsg, devices_msg);
    }
    return devices_msg[type];
  }
  static createCategory(type, value) {
    logger.info(`categoryModel.createCategory: ${type}`);
    if (!devices_msg.hasOwnProperty(value)) {
      devices_msg[type] = value;
      writeDataFile(devicesMsg, devices_msg);
      return devices_msg[type];
    }
    return null;
  }
  static deleteCategory(type) {
    logger.info(`categoryModel.deleteCategory: ${type}`);
    if (devices_msg.hasOwnProperty(type)) {
      delete devices_msg[type];
      writeDataFile(devicesMsg, devices_msg);
    }
    return devices_msg[type];
  }

  static getMessagesType() {
    return messages_types;
  }

  // number → canonical symbol (catalog dictionary). Used by the encoders.
  static symbolForValue(param, value) {
    return symbolForValue(param, value);
  }

  static getAtributes(type) {
    logger.debug(`categoryModel.getAtributes: ${type}`);
    const profile = resolveMissionProfile(type);
    if (!profile) return [];
    return Object.values(profile.params);
  }

  static getAttributesList(type) {
    logger.debug(`categoryModel.getAttributesList: ${type}`);
    const profile = resolveMissionProfile(type);
    if (!profile) return [];
    return Object.values(profile.params).map(formatParamDef);
  }

  // Per-waypoint parameter defs (mode_turn, speed, gimbal_pitch). Same rich shape
  // as getAttributesList; the UI renders these on each waypoint instead of the route.
  static getWaypointParams(type) {
    logger.debug(`categoryModel.getWaypointParams: ${type}`);
    const profile = resolveMissionProfile(type);
    if (!profile) return [];
    return Object.values(profile.waypointParams).map(formatParamDef);
  }

  // Default (wire number) for a per-waypoint param, used when a waypoint omits it.
  static getWaypointDefault(type, param) {
    const profile = resolveMissionProfile(type);
    return profile?.waypointParams?.[param]?.default ?? _mission_schema.waypoint_params?.[param]?.default ?? 0;
  }

  static getAttributesDefaults(type) {
    logger.debug(`categoryModel.getAttributesDefaults: ${type}`);
    const profile = resolveMissionProfile(type);
    if (!profile) return {};
    return Object.values(profile.params).reduce((acc, { id, default: defaultValue }) => {
      if (defaultValue !== undefined) acc[id] = defaultValue;
      return acc;
    }, {});
  }

  static getAtributesParam({ type, param }) {
    logger.debug(`categoryModel.getAtributesParam: ${type}-${param}`);
    const profile = resolveMissionProfile(type);
    if (!profile) return {};
    // Look in route params first, then per-waypoint params (e.g. mode_turn).
    const def = profile.params[param] ?? profile.waypointParams[param];
    const options = def?.options ?? [];
    // Keep the legacy {id,name} wire shape so SelectField needs no change.
    return options.map((opt) => ({ id: opt.value, name: opt.name }));
  }

  static getActions({ type }) {
    logger.debug(`categoryModel.getActions: ${type}`);
    const profile = resolveMissionProfile(type);
    if (!profile) return [];
    // Superset: legacy {name,id,param,description} + rich {key,label,payload}.
    return Object.entries(profile.actions).map(([name, a]) => ({
      name,
      id: a.id,
      key: a.key,
      label: a.label ?? name,
      description: a.description,
      param: a.payload != null, // derived back-compat flag
      payload: a.payload ?? null,
    }));
  }

  // {actionName: payload.default} — lets the client seed catalog defaults.
  static getActionDefaults(type) {
    logger.debug(`categoryModel.getActionDefaults: ${type}`);
    const profile = resolveMissionProfile(type);
    if (!profile) return {};
    return Object.entries(profile.actions).reduce((acc, [name, a]) => {
      if (a.payload?.default !== undefined) acc[name] = a.payload.default;
      return acc;
    }, {});
  }
}
