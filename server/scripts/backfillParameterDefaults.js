// One-off backfill: ElementItem/Base rows created before applyParameterDefaults
// was wired into the create paths never got their ElementType's
// parameterDefs.default values persisted into `attributes` — the client form
// only ever showed those defaults as a display-time fallback
// (`attributes?.[key] ?? def.default`), so `attributes` stayed null/partial
// and a GET returned nothing for those params. This fills in only the
// MISSING keys (never overwrites a value a user actually set) for rows whose
// type currently has parameterDefs with a `default`. Idempotent — a row with
// no missing default keys is left untouched.
//
// Usage: node server/scripts/backfillParameterDefaults.js

import sequelize from '../common/sequelize.js';
import { logger } from '../common/logger.js';
import { applyParameterDefaults } from '../models/markers/elementItems.js';
import { resolveEffectiveParameterDefs } from '../models/markers/elementTypes.js';

async function backfillElementItems(typeCache) {
  const items = await sequelize.models.ElementItem.findAll({
    include: [{ model: sequelize.models.ElementGroup, as: 'group' }],
  });
  let updated = 0;

  for (const item of items) {
    const typeId = item.group?.typeId;
    if (!typeId) continue;
    if (!typeCache.has(typeId)) {
      typeCache.set(typeId, await sequelize.models.ElementType.findByPk(typeId));
    }
    const parameterDefs = resolveEffectiveParameterDefs(typeCache.get(typeId));
    const merged = applyParameterDefaults(item.attributes, parameterDefs);
    const before = JSON.stringify(item.attributes ?? null);
    if (JSON.stringify(merged) === before) continue;

    item.attributes = merged;
    await item.save();
    updated += 1;
    logger.info(`ElementItem ${item.id}: backfilled parameter defaults`, { attributes: merged });
  }

  logger.info(`ElementItem: ${updated} rows backfilled`);
}

async function backfillBases(typeCache) {
  const bases = await sequelize.models.Base.findAll();
  let updated = 0;

  for (const base of bases) {
    if (!base.typeId) continue;
    if (!typeCache.has(base.typeId)) {
      typeCache.set(base.typeId, await sequelize.models.ElementType.findByPk(base.typeId));
    }
    const parameterDefs = resolveEffectiveParameterDefs(typeCache.get(base.typeId));
    const merged = applyParameterDefaults(base.attributes, parameterDefs);
    const before = JSON.stringify(base.attributes ?? null);
    if (JSON.stringify(merged) === before) continue;

    base.attributes = merged;
    await base.save();
    updated += 1;
    logger.info(`Base ${base.id}: backfilled parameter defaults`, { attributes: merged });
  }

  logger.info(`Base: ${updated} rows backfilled`);
}

async function main() {
  const typeCache = new Map();
  await backfillElementItems(typeCache);
  await backfillBases(typeCache);

  logger.info('Backfill complete');
  process.exit(0);
}

main().catch((error) => {
  logger.error(`Backfill failed: ${error.message}`, { stack: error.stack });
  process.exit(1);
});
