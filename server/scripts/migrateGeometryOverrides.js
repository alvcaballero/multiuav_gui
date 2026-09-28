// One-off migration: ElementItem/Base rows created before the geometry fix
// may still carry a per-instance `attributes.geometry` override (dimensions
// diverging from their ElementType, plus a `yaw`). Dimensions overrides are
// discarded outright (same-type-same-dimensions is now enforced by schema);
// a `yaw` value, if present and `azimFront` isn't already set, is rescued
// into the new `azimFront` column instead of being thrown away — it wasn't
// what caused the bug. Idempotent — safe to re-run (rows with no
// `attributes.geometry` are left untouched).
//
// Usage: node server/scripts/migrateGeometryOverrides.js

import sequelize from '../common/sequelize.js';
import { logger } from '../common/logger.js';

async function migrateRows(model, label) {
  const rows = await model.findAll();
  let rescued = 0;
  let cleaned = 0;

  for (const row of rows) {
    const geometry = row.attributes?.geometry;
    if (!geometry) continue;

    if (row.azimFront == null && typeof geometry.yaw === 'number') {
      row.azimFront = geometry.yaw;
      rescued += 1;
    }

    const { geometry: _dropped, ...rest } = row.attributes;
    row.attributes = Object.keys(rest).length > 0 ? rest : null;
    cleaned += 1;

    await row.save();
    logger.info(`${label} ${row.id}: dropped attributes.geometry`, {
      azimFrontRescued: row.azimFront,
    });
  }

  logger.info(`${label}: ${cleaned} rows cleaned, ${rescued} azimFront values rescued from yaw`);
}

async function main() {
  await migrateRows(sequelize.models.ElementItem, 'ElementItem');
  await migrateRows(sequelize.models.Base, 'Base');

  logger.info('Migration complete');
  process.exit(0);
}

main().catch((error) => {
  logger.error(`Migration failed: ${error.message}`, { stack: error.stack });
  process.exit(1);
});
