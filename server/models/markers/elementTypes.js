import path from 'path';
import fs from 'fs';
import { fileURLToPath } from 'url';
import { parse } from 'yaml';
import sequelize from '../../common/sequelize.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ASSETS_DIR = path.resolve(__dirname, '../../data/element-types');

// Asset folders are named `<id>-<slug>` (e.g. `12-wind-turbine`) — the id
// prefix is what code ever resolves by, the slug suffix exists purely so a
// human browsing server/data/element-types/ can tell what's in each folder
// without cross-referencing the DB. Never parsed back into anything.
function slugify(name) {
  return name
    .toLowerCase()
    .normalize('NFD')
    .replace(/[̀-ͯ]/g, '') // strip accents (á -> a)
    .replace(/[^a-z0-9]+/g, '-')
    .replace(/^-+|-+$/g, '');
}

// Finds the existing asset folder for `id` regardless of its slug suffix.
// Bounded match (`(-.*)?$`) so id 1 can't accidentally match a folder named
// 10-something/11-something/etc.
function findAssetDirName(id) {
  if (!fs.existsSync(ASSETS_DIR)) return null;
  const boundary = new RegExp(`^${id}(-.*)?$`);
  return fs.readdirSync(ASSETS_DIR).find((entry) => boundary.test(entry)) ?? null;
}

// A type's stored semantic/parametric model file carries its adjustable
// state in one of two shapes, depending on which generation produced it:
//   - wtsem-type/0.2, key `state_defaults`: flat {key: scalar}, e.g.
//     `nacelle_heading_deg: 240`.
//   - insem/0.2, key `state`: {key: scalar} OR {key: {value, unit?, limits?,
//     description?, ...}} — the two forms are MIXED within the same file
//     (see goliath_crane.insem.yaml's `operational_status: parked` sitting
//     alongside fully-structured entries), so each entry is inspected on its
//     own, not assumed uniform.
// Either way this maps to our ParameterDefSchema shape. Data type is
// inferred from the JS type of the default value — neither source format
// carries an enum option list, so a string value never becomes dataType
// 'enum' here, only a human editing parameterDefs in the DB can add that.
function humanizeKey(key) {
  return key.replace(/_/g, ' ').replace(/^./, (c) => c.toUpperCase());
}

function dataTypeOf(value) {
  return typeof value === 'number' ? 'number' : typeof value === 'boolean' ? 'boolean' : 'string';
}

function isStructuredStateEntry(entry) {
  return entry !== null && typeof entry === 'object' && !Array.isArray(entry) && 'value' in entry;
}

function stateToParameterDefs(state) {
  if (!state || typeof state !== 'object') return undefined;
  return Object.entries(state).map(([key, entry]) => {
    if (isStructuredStateEntry(entry)) {
      const def = {
        key,
        label: humanizeKey(key),
        dataType: dataTypeOf(entry.value),
        default: entry.value,
      };
      if (typeof entry.unit === 'string' && entry.unit.trim()) def.unit = entry.unit.trim();
      if (typeof entry.description === 'string' && entry.description.trim()) def.description = entry.description;
      if (Array.isArray(entry.limits) && entry.limits.length === 2) {
        const [min, max] = entry.limits;
        if (typeof min === 'number') def.min = min;
        if (typeof max === 'number') def.max = max;
      }
      return def;
    }
    return { key, label: humanizeKey(key), dataType: dataTypeOf(entry), default: entry };
  });
}

// Parses this type's stored definition file (if any) and derives
// parameterDefs from its `state` (insem/0.2) or `state_defaults`
// (wtsem-type/0.2). Never throws — a missing/unparseable file just means
// there's nothing to derive.
function parameterDefsFromDefinitionFile(id) {
  const filePath = elementTypesModel.getAssetPath(id, 'definition');
  if (!filePath) return undefined;
  try {
    // merge: true — some definition files (e.g. windTurbine's blade_A/B/C)
    // use YAML merge keys elsewhere in the document; parsing consistently
    // with definitionResolver.js avoids surprises if `state`/`parameters`
    // ever end up behind one too.
    const parsed = parse(fs.readFileSync(filePath, 'utf8'), { merge: true });
    return stateToParameterDefs(parsed?.state ?? parsed?.state_defaults);
  } catch {
    return undefined;
  }
}

// The effective parameterDefs for a type: the DB copy (`attributes.
// parameterDefs`, authored via the form-builder) when present, else derived
// from the type's stored .type.yaml file, else undefined. Never writes
// anything back — this is a read-only, computed view, not a migration.
export function resolveEffectiveParameterDefs(type) {
  const dbDefs = type?.attributes?.parameterDefs;
  // An empty array counts as "nothing defined yet", same as null/undefined —
  // otherwise a type saved once before it had any parameters (empty list)
  // would be stuck ignoring its definition file forever.
  if (dbDefs && dbDefs.length > 0) return dbDefs;
  if (!type?.id) return undefined;
  return parameterDefsFromDefinitionFile(type.id);
}

export const elementTypesModel = {
  // Merges in file-derived parameterDefs when the DB copy is empty — used by
  // every consumer that RESOLVES a type for actual use (rendering, item
  // creation/validation). getById() deliberately does NOT do this merge: it
  // backs the type editor's own fetch, and round-tripping a derived value
  // back through a save would silently persist it as if hand-authored,
  // permanently defeating the "file is a fallback, not a default" contract.
  async getAll() {
    const types = await sequelize.models.ElementType.findAll();
    return types.map((type) => {
      const plain = type.get({ plain: true });
      if (!plain.attributes?.parameterDefs?.length) {
        const derived = parameterDefsFromDefinitionFile(plain.id);
        if (derived) plain.attributes = { ...(plain.attributes || {}), parameterDefs: derived };
      }
      return plain;
    });
  },

  async getById(id) {
    return await sequelize.models.ElementType.findOne({ where: { id } });
  },

  async create(elementType) {
    // `id` is autoincrement now — any id the caller sends (e.g. a stale
    // client re-sending a fetched object) is ignored, never honored, so
    // there's no path to an id collision or a client picking its own PK.
    const { name, description, icon, model3d, definitionYaml, color, isCustom, attributes } = elementType;

    return await sequelize.models.ElementType.create({
      name,
      description: description || '',
      icon: icon ?? null,
      model3d: model3d ?? null,
      definitionYaml: definitionYaml ?? null,
      color: color ?? null,
      isCustom: isCustom ?? false,
      attributes: attributes ?? null,
    });
  },

  async update(id, elementType) {
    const { name, description, icon, model3d, definitionYaml, color, isCustom, attributes } = elementType;
    const myType = await sequelize.models.ElementType.findOne({ where: { id } });
    if (!myType) {
      return null;
    }
    const nameChanged = name && name !== myType.name;
    if (name) myType.name = name;
    if (description !== undefined) myType.description = description;
    if (icon !== undefined) myType.icon = icon;
    if (model3d !== undefined) myType.model3d = model3d;
    if (definitionYaml !== undefined) myType.definitionYaml = definitionYaml;
    if (color !== undefined) myType.color = color;
    if (isCustom !== undefined) myType.isCustom = isCustom;
    if (attributes !== undefined) myType.attributes = attributes;
    await myType.save();
    // Keep the asset folder's human-readable suffix in sync with the type's
    // current name — a no-op when there's no folder yet (nothing to rename).
    if (nameChanged) await this.ensureAssetDir(id, myType.name);
    return myType;
  },

  async delete(id) {
    return await sequelize.models.ElementType.destroy({ where: { id } });
  },

  // ─── Assets (icon/model files) ────────────────────────────────────────────
  // Filesystem storage, not SQL — kept here (not in a separate module)
  // because it's small and only ever used alongside the type catalog CRUD.

  getAssetPath(id, assetType) {
    const ext =
      assetType === 'icon' ? ['png', 'svg', 'jpg'] : assetType === 'definition' ? ['yaml', 'yml'] : ['glb', 'gltf'];
    const dirName = findAssetDirName(id);
    if (!dirName) return null;
    const dir = path.join(ASSETS_DIR, dirName);

    for (const e of ext) {
      const p = path.join(dir, `${assetType}.${e}`);
      if (fs.existsSync(p)) return p;
    }
    return null;
  },

  // Async now — resolving/keeping the `<id>-<slug>` suffix in sync needs the
  // type's current `name`, looked up here when the caller doesn't already
  // have it (e.g. multer's destination callback only has the route's raw
  // id). Renames an existing folder whose slug has drifted from `name`
  // (see update() above); creates a fresh one otherwise. Falls back to a
  // bare `<id>` folder if `name` can't be resolved (no matching DB row —
  // shouldn't happen outside a race, but asset storage must never throw).
  async ensureAssetDir(id, name) {
    let resolvedName = name;
    if (resolvedName === undefined) {
      const type = await sequelize.models.ElementType.findByPk(id);
      resolvedName = type?.name;
    }
    const expected = resolvedName ? `${id}-${slugify(resolvedName)}` : String(id);
    const existing = findAssetDirName(id);

    if (!existing) {
      const dir = path.join(ASSETS_DIR, expected);
      fs.mkdirSync(dir, { recursive: true });
      return dir;
    }
    if (existing !== expected && resolvedName) {
      const oldDir = path.join(ASSETS_DIR, existing);
      const newDir = path.join(ASSETS_DIR, expected);
      fs.renameSync(oldDir, newDir);
      return newDir;
    }
    return path.join(ASSETS_DIR, existing);
  },
};
