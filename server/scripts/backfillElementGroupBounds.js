// One-off: computes and persists minLat/maxLat/minLng/maxLng on every
// existing ElementGroup, since those columns are only kept up to date going
// forward (elementItemsModel.create/update/delete and
// markersModel._upsertElements now call elementGroupsModel.recalculateBounds).
// Idempotent — safe to re-run, always overwrites to the current computed value.
//
// Usage: node server/scripts/backfillElementGroupBounds.js

import sequelize from '../common/sequelize.js';
import { logger } from '../common/logger.js';
import { elementGroupsModel } from '../models/markers/elementGroups.js';

async function main() {
  const groups = await sequelize.models.ElementGroup.findAll({ attributes: ['id'], raw: true });

  for (const { id } of groups) {
    await elementGroupsModel.recalculateBounds(id);
  }

  logger.info(`Backfilled bounds on ${groups.length} element groups`);
  process.exit(0);
}

main().catch((error) => {
  logger.error(`backfillElementGroupBounds failed: ${error.message}`, { stack: error.stack });
  process.exit(1);
});
