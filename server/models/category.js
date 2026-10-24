import { readDataFile } from '../common/utils.js';
import { interfacesDir, missionSchema } from '../config/config.js';
import { KNOWN_SERVICE_KEYS, KNOWN_SUBSCRIBER_KEYS, KNOWN_PUBLISHER_KEYS, ServiceKey } from '../config/deviceMsgCatalog.js';
import { logger } from '../common/logger.js';
import { symbolsForService, profileFor } from './mission/missionSymbols.js';
import { hasDecoder, decodableMsgTypes } from './msgDecode.js';
import { commandSchema, validateCommandSchema, validateCommandOverrides } from './commands/commandSchema.js';
import { CommandType } from '../config/commandTypes.js';
import { commandRegistry } from './commands/commandRegistry.js';
import { endpointsOf } from './commands/commandEncoders.js';
import { buildCatalog, formatProblem, loadInterfaces, MESSAGE_FORMAT, messageFormatOf, rootOf } from './interfaces/interfaceCatalog.js';
import { deleteInterfaceFile, isInterfaceKey, INTERFACE_KEY, writeInterfaceFile } from './interfaces/interfaceStore.js';

export { MESSAGE_FORMAT, messageFormatOf };

// "devices_msg" = the interface catalog: one file per interface in config/devices/interfaces/
// (models/interfaces/). `rawInterfaces` is what the files contain; `devices_msg` the interfaces
// that passed validation, which is what everything else reads. An invalid file is left out (the
// others keep working) and its devices get no telemetry: the boot reports it, never in silence.
const loaded = loadInterfaces(interfacesDir);
let rawInterfaces = loaded.raw;
let devices_msg = loaded.catalog;
let abstractInterfaces = loaded.abstract;
let interfaceProblems = loaded.problems;
const _mission_schema = readDataFile(missionSchema);

function reportInterfaceProblems(problems) {
  if (problems.length === 0) return;
  const byInterface = new Map();
  for (const p of problems) {
    const key = String(p.path[0] ?? '');
    byInterface.set(key, [...(byInterface.get(key) ?? []), p]);
  }
  for (const [key, list] of byInterface) {
    logger.error(`interface '${key}' not loaded: ${list.map(formatProblem).join('; ')}`);
  }
  if (process.env.LOG_LEVEL === 'debug') throw new Error(`interfaces: ${problems.map(formatProblem).join('; ')}`);
}

reportInterfaceProblems(interfaceProblems);

/** A change to the catalog the API refuses: `errors` are {path, message}, as zod reports them. */
export class CategoryError extends Error {
  constructor(status, message, errors = []) {
    super(message);
    this.status = status;
    this.errors = errors;
  }
}

// Valida que cada key de los bloques subscribers/publishers/services/actions en
// devices_msg esté registrada en su catálogo correspondiente. Una key huérfana
// significa que un UAV declara una capacidad que el sistema no sabe manejar:
//   - subscribers → key sin entrada en deviceMsgCatalog (posible typo o topic no soportado)
//   - publishers/services/actions → sin entrada en su catálogo → comando invisible o roto
//
// Cada bloque se valida contra SU SSOT (telemetría ≠ comandos: naturalezas distintas).
// subscribers → SubscriberKey (telemetría); publishers → PublisherKey (comandos que
// publican); services/actions → ServiceKey. Todas en snake_case.
// warn por defecto; en modo debug (LOG_LEVEL=debug) lanza para forzar la alineación
// temprano — un typo en el YAML no debe llegar silencioso a runtime.
const DEVICES_MSG_BLOCKS = {
  subscribers: KNOWN_SUBSCRIBER_KEYS,
  publishers: KNOWN_PUBLISHER_KEYS,
  services: KNOWN_SERVICE_KEYS,
  actions: KNOWN_SERVICE_KEYS,
};

// Además de la key, cada subscriber valida el PAR (slot, messageType) contra el registry
// de msgDecode: un messageType mal escrito en el YAML (o en el decoder) dejaría el topic
// suscripto y mudo, sin ningún error.
function validateDevicesMsgKeys(catalog, { strict = process.env.LOG_LEVEL === 'debug' } = {}) {
  const orphans = [];
  const undecoded = [];
  for (const [category, def] of Object.entries(catalog)) {
    for (const [block, knownKeys] of Object.entries(DEVICES_MSG_BLOCKS)) {
      for (const key of Object.keys(def?.[block] ?? {})) {
        if (!knownKeys.has(key)) orphans.push(`${category}.${block}.${key}`);
      }
    }
    for (const [slot, entry] of Object.entries(def?.subscribers ?? {})) {
      if (!hasDecoder(slot, entry?.messageType)) undecoded.push(`${category}.subscribers.${slot} (${entry?.messageType})`);
    }
  }
  const problems = [];
  if (orphans.length) problems.push(`${orphans.length} unknown key(s) not in catalog: ${orphans.join(', ')}`);
  if (undecoded.length) problems.push(`${undecoded.length} subscriber(s) with no decoder for their messageType: ${undecoded.join(', ')}`);
  if (problems.length === 0) return;
  const msg = `devices_msg: ${problems.join('; ')}`;
  if (strict) throw new Error(msg);
  logger.warn(msg);
}

validateDevicesMsgKeys(devices_msg);

// (The old `serviceType`/`actionType` fields, and any other key nobody reads, are rejected by the
// file schema, schemas/zod/interface.js: that interface is not loaded.)

// command_schema.yaml: una entrada por CommandType, y los bloques `commands:` de
// devices_msg solo restringen ese dominio. commandRegistry: cada command de una categoría
// sale por UN endpoint (dos matches sin `commands.<Type>.via` es ambiguo, y un `via` tiene
// que apuntar a un endpoint que lo implemente). Misma política: warn, o throw en debug.
// (Que el msgType de ese endpoint tenga encoder lo valida commands.js: msgEncode importa los
// encoders de misión, que importan este módulo, así que acá no se puede.)
function validateCommands(catalog, { strict = process.env.LOG_LEVEL === 'debug', schema = true } = {}) {
  const problems = [
    ...(schema ? validateCommandSchema(commandSchema, Object.values(CommandType)) : []),
    ...validateCommandOverrides(commandSchema, catalog),
    ...Object.entries(catalog).flatMap(([category, def]) =>
      commandRegistry.resolveCommands(def).problems.map((p) => `${category}.${p}`)
    ),
  ];
  if (problems.length === 0) return;
  const msg = `command_schema: ${problems.length} problem(s): ${problems.join('; ')}`;
  if (strict) throw new Error(msg);
  logger.warn(msg);
}

validateCommands(devices_msg);

// Applies a change to ONE interface file (def null = delete) only if the whole catalog still
// validates: an edit must not leave an invalid file on disk, nor break another interface. The
// problems some other interface already had (it was not loaded) do not block it.
async function commitInterface(key, def) {
  if (!isInterfaceKey(key)) {
    throw new CategoryError(400, `'${key}' is not a valid category name`, [
      { path: [], message: `must match ${INTERFACE_KEY}` },
    ]);
  }
  const candidate = { ...rawInterfaces };
  if (def === null) delete candidate[key];
  else candidate[key] = def;
  const { catalog, abstract, problems } = buildCatalog(candidate);
  const alreadyBroken = new Set(interfaceProblems.map((p) => String(p.path[0])));
  const introduced = problems.filter((p) => p.path[0] === key || !alreadyBroken.has(String(p.path[0])));
  if (introduced.length) {
    throw new CategoryError(400, `invalid category '${key}'`, introduced);
  }
  if (def === null) await deleteInterfaceFile(interfacesDir, key);
  else await writeInterfaceFile(interfacesDir, key, def);
  rawInterfaces = candidate;
  devices_msg = catalog;
  abstractInterfaces = abstract;
  interfaceProblems = problems;
  if (catalog[key]) {
    // Same warnings as at boot (never fatal here: the file is valid, it may just name a slot
    // or command this server does not know yet). An abstract base is checked through its children.
    validateDevicesMsgKeys({ [key]: catalog[key] }, { strict: false });
    validateCommands({ [key]: catalog[key] }, { strict: false, schema: false });
  }
}

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
// The profile is DERIVED from the configure_mission serviceType (see profileFor), so it
// can't contradict the encoder. A category whose firmware supports less than its serviceType
// implies says so with `mission:` in its file (dji_M210_*, px4_mavros*: v1; dji_M600: default),
// inherited through `extends`. No device/service → 'default', unfiltered, so the planning UI
// always has a coherent baseline.
function resolveMissionProfile(type) {
  const dev = devices_msg[type];
  const serviceType = dev?.services?.[ServiceKey.CONFIGURE_MISSION]?.messageType;
  const profileName = dev?.mission ?? profileFor(serviceType);
  const profile = _mission_schema.profiles?.[profileName] ?? _mission_schema.profiles?.default;
  if (!profile) {
    logger.warn(`mission_schema: no profile resolvable for category '${type}' (and no 'default' profile)`);
    return undefined;
  }

  // Encoder family (via configure_mission serviceType) decides supported options.
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

// The `camera` slot is only subscribed when the device has a Websocket camera AND the category
// gives it a messageType. Shared by the ROS subscription and the telemetry watchdog, so the
// watchdog never expects a slot nobody subscribed.
export function subscribesCamera(subscribers, camera) {
  const hasWebsocketCamera = (camera ?? []).some((cam) => cam.type == 'Websocket');
  return hasWebsocketCamera && Boolean(subscribers?.['camera']?.['messageType']);
}

/** The message format a protocol carries (MESSAGE_FORMAT / messageFormatOf: interfaceCatalog.js). */
export const messageFormatOfProtocol = (protocol) => (protocol === 'mavlink' ? MESSAGE_FORMAT.MAVLINK : MESSAGE_FORMAT.ROS_MSG);

export class categoryModel {
  /** Every category, or only those a device of `protocol` can use. */
  static getAll({ protocol } = {}) {
    logger.debug(`categoryModel.getAll protocol=${protocol ?? '*'}`);
    const names = Object.keys(devices_msg);
    if (!protocol) return names;
    return names.filter((name) => messageFormatOf(devices_msg[name]) === messageFormatOfProtocol(protocol));
  }

  /**
   * getAll with what the device form needs to group variants: `root` is the top of the `extends`
   * chain (dji_M210_melodic → dji_M210_noetic), sorted so each root comes before its variants.
   */
  static getAllDetailed({ protocol } = {}) {
    return this.getAll({ protocol })
      .map((name) => {
        const { firmwares, vehicleTypes, control } = devices_msg[name];
        const messageFormat = messageFormatOf(devices_msg[name]);
        return { name, root: rootOf(rawInterfaces, name), messageFormat, firmwares, vehicleTypes, control };
      })
      .sort((a, b) => a.root.localeCompare(b.root) || (a.name !== a.root) - (b.name !== b.root) || a.name.localeCompare(b.name));
  }

  /**
   * Why a device with this protocol/category cannot work, or null. A mavlink device on a
   * ROS category (or the other way) would get no telemetry and no commands, with no error.
   * @returns {{path: string[], message: string}|null}
   */
  static deviceProblem({ protocol = 'ros', category }) {
    const def = devices_msg[category];
    if (abstractInterfaces[category]) {
      return { path: ['category'], message: `category '${category}' is abstract: a base to extend, not for a device` };
    }
    if (!def) return { path: ['category'], message: `unknown category '${category}'` };
    const format = messageFormatOf(def);
    if (format !== messageFormatOfProtocol(protocol)) {
      const speaks = format === MESSAGE_FORMAT.MAVLINK ? 'MAVLink' : 'ROS';
      return {
        path: ['category'],
        message: `category '${category}' speaks ${speaks} messages, which protocol '${protocol}' does not carry`,
      };
    }
    return null;
  }
  static getCategory(type) {
    logger.debug(`categoryModel.getCategory: ${type}`);
    return devices_msg[type];
  }
  /**
   * The category as its FILE says it (with `extends`, only its own entries), for the editor: saving
   * the resolved one would copy every inherited entry into the file and cut it off from its parent.
   */
  static getCategoryRaw(type) {
    return rawInterfaces[type];
  }
  /** Every loaded interface, key → resolved definition (what the API and the registries see). */
  static getCatalog() {
    return devices_msg;
  }
  /** Why each interface that was not loaded was refused (path[0] is its key). */
  static getProblems() {
    return interfaceProblems;
  }

  // An interface exists when it has a file, even one that did not validate: the API may fix it.
  static #exists(type) {
    return Object.hasOwn(rawInterfaces, type) || interfaceProblems.some((p) => p.path[0] === type);
  }

  /** @throws {CategoryError} 404 if it does not exist, 400 if `value` does not validate */
  static async updateCategory(type, value) {
    logger.info(`categoryModel.updateCategory: ${type}`);
    if (!this.#exists(type)) throw new CategoryError(404, `unknown category '${type}'`);
    await commitInterface(type, value);
    return devices_msg[type];
  }
  /** @throws {CategoryError} 409 if it already exists, 400 if `value` does not validate */
  static async createCategory(type, value) {
    logger.info(`categoryModel.createCategory: ${type}`);
    if (this.#exists(type)) throw new CategoryError(409, `category '${type}' already exists`);
    await commitInterface(type, value);
    return devices_msg[type];
  }
  /** @throws {CategoryError} 404 if it does not exist */
  static async deleteCategory(type) {
    logger.info(`categoryModel.deleteCategory: ${type}`);
    if (!this.#exists(type)) throw new CategoryError(404, `unknown category '${type}'`);
    await commitInterface(type, null);
  }

  // Message types the category editor offers, per block and key (GET /api/category/messages).
  // DERIVED, never a hand-kept list (it was messages.yaml, and it drifted):
  //   subscribers → what msgDecode decodes, plus what categories declare (wildcard slots
  //                 such as battery accept any type, so only declarations name one)
  //   services / actions / publishers → what categories declare; every one of them is
  //                 checked at boot against commandRegistry/msgEncode. A command matched by key
  //                 accepts any msgType, so the registries can't enumerate these.
  // Read from the in-memory devices_msg, so a category edited at runtime shows up.
  static getMessagesType() {
    const catalog = { subscribers: {}, publishers: {}, services: {}, actions: {} };
    const add = (block, key, msgType) => {
      if (!msgType) return;
      const types = (catalog[block][key] ??= []);
      if (!types.includes(msgType)) types.push(msgType);
    };
    for (const [slot, types] of decodableMsgTypes()) for (const msgType of types) add('subscribers', slot, msgType);
    for (const def of Object.values(devices_msg)) {
      for (const [slot, entry] of Object.entries(def?.subscribers ?? {})) add('subscribers', slot, entry?.messageType);
      for (const { block, key, msgType } of endpointsOf(def)) add(block, key, msgType);
    }
    for (const block of Object.values(catalog)) for (const types of Object.values(block)) types.sort();
    return catalog;
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
