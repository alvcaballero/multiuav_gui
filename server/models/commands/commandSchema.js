// Command schema — contrato de dominio de cada command (config/devices/command_schema.yaml)
// resuelto para un device concreto.
//
// Tres piezas, mismo patrón que mission_schema:
//   - command_schema.yaml: qué significa cada command (params, unidades, rangos, label…).
//   - devices_msg.yaml › <categoría> › commands: límites del dron; solo RESTRINGE.
//   - resolveCommand(): junta las dos para un device → lo que /api/commands/types entrega
//     y (fase 6b) contra lo que sendCommand valida.
//
// Las funciones reciben el schema y la config de categoría como argumentos (puras,
// testeables); `commandSchema` es la instancia cargada del archivo.

import { readDataFile } from '../../common/utils.js';
import { commandSchema as commandSchemaFile } from '../../config/config.js';

export const commandSchema = readDataFile(commandSchemaFile);

const PARAM_TYPES = new Set(['number', 'string', 'boolean', 'datetime', 'select']);
const GROUPS = new Set(['flight', 'mission', 'gimbal', 'camera', 'threat', 'system']);
const SEVERITIES = new Set(['normal', 'danger']);
// Lo único que un override por categoría puede tocar de un param.
const OVERRIDABLE = new Set(['min', 'max', 'step', 'default', 'options']);

const inRange = (value, { min, max }) => (min == null || value >= min) && (max == null || value <= max);

function paramProblems(where, param) {
  const problems = [];
  if (!PARAM_TYPES.has(param.type)) problems.push(`${where}: unknown type '${param.type}'`);
  if (param.min != null && param.max != null && param.min > param.max) problems.push(`${where}: min > max`);
  if (param.type === 'number' && param.default != null && !inRange(param.default, param)) {
    problems.push(`${where}: default ${param.default} out of [${param.min ?? ''}, ${param.max ?? ''}]`);
  }
  if (param.type === 'select') {
    if (!param.options?.length) problems.push(`${where}: select without options`);
    else if (param.default != null && !param.options.some((o) => o.value === param.default)) {
      problems.push(`${where}: default ${param.default} is not one of its options`);
    }
  }
  return problems;
}

/**
 * Problemas del schema de dominio: un CommandType sin entrada, una entrada que no es
 * un CommandType, o un command/param mal formado. [] si está todo bien.
 * @param {object} schema       command_schema.yaml parseado
 * @param {string[]} commandTypes valores de CommandType
 */
export function validateCommandSchema(schema, commandTypes) {
  const commands = schema?.commands ?? {};
  const problems = [];
  for (const type of commandTypes) {
    if (!commands[type]) problems.push(`${type}: no entry in command_schema`);
  }
  const known = new Set(commandTypes);
  for (const [type, def] of Object.entries(commands)) {
    if (!known.has(type)) problems.push(`${type}: not a CommandType`);
    if (!def.label) problems.push(`${type}: missing label`);
    if (!GROUPS.has(def.group)) problems.push(`${type}: unknown group '${def.group}'`);
    if (def.severity != null && !SEVERITIES.has(def.severity)) {
      problems.push(`${type}: unknown severity '${def.severity}'`);
    }
    for (const [id, param] of Object.entries(def.params ?? {})) {
      problems.push(...paramProblems(`${type}.${id}`, param));
    }
  }
  return problems;
}

// Aplica el override de una categoría a un param. Lo que no se puede restringir queda igual.
function mergeParam(param, override = {}) {
  return { ...param, ...Object.fromEntries(Object.entries(override).filter(([k]) => OVERRIDABLE.has(k))) };
}

/**
 * Problemas de los bloques `commands:` de devices_msg: command o param desconocido, un
 * campo que no se puede overridear, o un override que AMPLÍA el dominio en vez de restringirlo.
 * @param {object} schema     command_schema.yaml parseado
 * @param {object} devicesMsg devices_msg.yaml parseado
 */
export function validateCommandOverrides(schema, devicesMsg) {
  const commands = schema?.commands ?? {};
  const problems = [];
  for (const [category, def] of Object.entries(devicesMsg ?? {})) {
    for (const [type, override] of Object.entries(def?.commands ?? {})) {
      const where = `${category}.commands.${type}`;
      const domain = commands[type];
      if (!domain) {
        problems.push(`${where}: unknown command`);
        continue;
      }
      for (const [id, paramOverride] of Object.entries(override?.params ?? {})) {
        const param = domain.params?.[id];
        if (!param) {
          problems.push(`${where}.${id}: unknown param`);
          continue;
        }
        for (const key of Object.keys(paramOverride)) {
          if (!OVERRIDABLE.has(key)) problems.push(`${where}.${id}: '${key}' can't be overridden`);
        }
        const merged = mergeParam(param, paramOverride);
        if (param.min != null && merged.min < param.min) problems.push(`${where}.${id}: min widens ${param.min}`);
        if (param.max != null && merged.max > param.max) problems.push(`${where}.${id}: max widens ${param.max}`);
        if (paramOverride.options) {
          const allowed = new Set((param.options ?? []).map((o) => o.value));
          if (paramOverride.options.some((o) => !allowed.has(o.value))) {
            problems.push(`${where}.${id}: options add values the domain doesn't have`);
          }
        }
        problems.push(...paramProblems(`${where}.${id}`, merged));
      }
    }
  }
  return problems;
}

// Forma de un param en el wire: la misma que los params de misión (formatParamDef en
// category.js) + `required`, así la UI renderiza los dos con el mismo componente.
function formatParam(id, p) {
  return {
    id,
    name: p.name ?? id,
    type: p.type,
    default: p.default ?? null,
    description: p.description ?? null,
    unit: p.unit ?? null,
    min: p.min ?? null,
    max: p.max ?? null,
    step: p.step ?? null,
    options: p.options ?? null,
    required: Boolean(p.required),
  };
}

// Un valor crudo (JSON, o el string de un input) → valor del param, o { error }.
function coerce(param, raw) {
  switch (param.type) {
    case 'number': {
      const value = Number(raw);
      if (typeof raw === 'boolean' || !Number.isFinite(value)) return { error: `must be a number, got '${raw}'` };
      if (param.min != null && value < param.min) return { error: `must be >= ${param.min}, got ${value}` };
      if (param.max != null && value > param.max) return { error: `must be <= ${param.max}, got ${value}` };
      return { value };
    }
    case 'select': {
      const option = param.options.find((o) => String(o.value) === String(raw));
      if (!option) return { error: `must be one of ${param.options.map((o) => o.value).join(', ')}, got '${raw}'` };
      return { value: option.value };
    }
    case 'boolean':
      if (raw === true || raw === 'true') return { value: true };
      if (raw === false || raw === 'false') return { value: false };
      return { error: `must be true or false, got '${raw}'` };
    case 'datetime':
      if (Number.isNaN(Date.parse(raw))) return { error: `must be a date, got '${raw}'` };
      return { value: raw };
    default:
      if (typeof raw !== 'string') return { error: `must be a string, got '${raw}'` };
      return { value: raw };
  }
}

/**
 * Valida los attributes de un send contra el command resuelto para el device.
 * Fuera de rango, de tipo equivocado, desconocido o required faltante → se rechaza (no
 * se recorta). Un param que no viene toma su default; sin default queda ausente (el
 * handler decide, ej. GimbalUAV completa con la telemetría).
 * @param {object} command    resultado de resolveCommand
 * @param {object} [attributes]
 * @returns {{ ok: true, values: object } | { ok: false, msg: string }} msg legible, nombra los params
 */
export function validateAttributes(command, attributes) {
  if (attributes != null && (typeof attributes !== 'object' || Array.isArray(attributes))) {
    return { ok: false, msg: `${command.type}: attributes must be an object` };
  }
  const input = attributes ?? {};
  const params = new Map(command.params.map((p) => [p.id, p]));
  const errors = Object.keys(input)
    .filter((key) => !params.has(key))
    .map((key) => `'${key}' is not a parameter`);
  const values = {};
  for (const param of params.values()) {
    const raw = input[param.id];
    if (raw == null || raw === '') {
      if (param.required) errors.push(`'${param.id}' is required`);
      else if (param.default != null) values[param.id] = param.default;
      continue;
    }
    const { value, error } = coerce(param, raw);
    if (error) errors.push(`'${param.id}' ${error}`);
    else values[param.id] = value;
  }
  if (errors.length) return { ok: false, msg: `${command.type}: ${errors.join('; ')}` };
  return { ok: true, values };
}

/**
 * Un command resuelto para una categoría: dominio + overrides de la categoría.
 * undefined si el schema no conoce el type.
 * @param {object} schema         command_schema.yaml parseado
 * @param {string} type           CommandType
 * @param {object} [categoryConfig] entrada de la categoría en devices_msg.yaml
 */
export function resolveCommand(schema, type, categoryConfig) {
  const def = schema?.commands?.[type];
  if (!def) return undefined;
  const overrides = categoryConfig?.commands?.[type]?.params ?? {};
  return {
    type,
    label: def.label,
    description: def.description ?? '',
    group: def.group,
    severity: def.severity ?? 'normal',
    confirm: def.confirm ?? null,
    hidden: Boolean(def.hidden),
    freeform: Boolean(def.freeform),
    params: Object.entries(def.params ?? {}).map(([id, p]) => formatParam(id, mergeParam(p, overrides[id]))),
  };
}
