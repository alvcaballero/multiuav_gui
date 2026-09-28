// One-off data fix: TypeParameterSchemaEditor.jsx (the ElementType parameter
// form-builder) stored `parameterDefs[].default` as whatever string a
// TextField handed back, never cast to `dataType` — so a `dataType: 'number'`
// def could carry `default: "240"` (string) instead of `240`. That string
// then propagated into every ElementItem/Base's `attributes` via
// applyParameterDefaults (create-time defaults) or backfillParameterDefaults
// (the one-off backfill for pre-existing rows). Both stages get normalized
// here — the ElementType's own parameterDefs, then every ElementItem/Base
// attribute value that matches one of those defs. Idempotent — a value
// that's already the right JS type is left untouched.
//
// Usage: node server/scripts/castParameterDataTypes.js

import sequelize from '../common/sequelize.js';
import { logger } from '../common/logger.js';

function castValue(raw, dataType) {
  if (raw === '' || raw == null) return raw;
  if (dataType === 'number') {
    if (typeof raw === 'number') return raw;
    const num = Number(raw);
    return Number.isFinite(num) ? num : raw;
  }
  if (dataType === 'boolean') {
    if (typeof raw === 'boolean') return raw;
    if (raw === 'true') return true;
    if (raw === 'false') return false;
    return raw;
  }
  return raw;
}

async function castTypeDefaults() {
  const types = await sequelize.models.ElementType.findAll();
  let updated = 0;

  for (const type of types) {
    const defs = type.attributes?.parameterDefs;
    if (!Array.isArray(defs) || defs.length === 0) continue;

    let changed = false;
    const castDefs = defs.map((def) => {
      const castedDefault = castValue(def.default, def.dataType);
      if (castedDefault !== def.default) changed = true;
      return { ...def, default: castedDefault };
    });
    if (!changed) continue;

    type.attributes = { ...type.attributes, parameterDefs: castDefs };
    await type.save();
    updated += 1;
    logger.info(`ElementType ${type.id}: cast parameterDefs.default to their declared dataType`, {
      parameterDefs: castDefs,
    });
  }

  logger.info(`ElementType: ${updated} rows fixed`);
}

async function castInstanceAttributes(model, label, resolveTypeId) {
  const rows = await model.findAll();
  const typeCache = new Map();
  let updated = 0;

  for (const row of rows) {
    if (!row.attributes) continue;
    const typeId = await resolveTypeId(row);
    if (!typeId) continue;
    if (!typeCache.has(typeId)) {
      typeCache.set(typeId, await sequelize.models.ElementType.findByPk(typeId));
    }
    const defs = typeCache.get(typeId)?.attributes?.parameterDefs;
    if (!Array.isArray(defs) || defs.length === 0) continue;

    const dataTypeByKey = new Map(defs.map((def) => [def.key, def.dataType]));
    let changed = false;
    const castAttributes = { ...row.attributes };
    for (const [key, value] of Object.entries(castAttributes)) {
      if (!dataTypeByKey.has(key)) continue;
      const castValueResult = castValue(value, dataTypeByKey.get(key));
      if (castValueResult !== value) {
        castAttributes[key] = castValueResult;
        changed = true;
      }
    }
    if (!changed) continue;

    row.attributes = castAttributes;
    await row.save();
    updated += 1;
    logger.info(`${label} ${row.id}: cast attribute values to their declared dataType`, {
      attributes: castAttributes,
    });
  }

  logger.info(`${label}: ${updated} rows fixed`);
}

async function main() {
  await castTypeDefaults();
  await castInstanceAttributes(sequelize.models.ElementItem, 'ElementItem', async (item) => {
    const group = await sequelize.models.ElementGroup.findByPk(item.groupId);
    return group?.typeId;
  });
  await castInstanceAttributes(sequelize.models.Base, 'Base', async (base) => base.typeId);

  logger.info('Cast complete');
  process.exit(0);
}

main().catch((error) => {
  logger.error(`Cast failed: ${error.message}`, { stack: error.stack });
  process.exit(1);
});
