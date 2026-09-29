// One-off fix: migrateElementTypeIdToInteger.js's first run (before this
// script existed) migrated the PK/FKs and renamed asset folders, but missed
// that icon/model3d/definitionYaml store a LITERAL URL
// (`/api/markers/types/<id>/icon`) baked in at upload time, not derived from
// the row's own id at read time — so every type kept pointing at its old
// string id's URL (now a renamed, nonexistent folder) after the swap.
// Idempotent — a URL that already matches the row's own id is left alone.
//
// Usage: node server/scripts/fixElementTypeAssetUrls.js

import sequelize from '../common/sequelize.js';
import { logger } from '../common/logger.js';

function rewrite(url, id) {
  if (!url) return url;
  return url.replace(/\/types\/[^/]+\//, `/types/${id}/`);
}

async function main() {
  const types = await sequelize.models.ElementType.findAll();
  let updated = 0;

  for (const type of types) {
    const icon = rewrite(type.icon, type.id);
    const model3d = rewrite(type.model3d, type.id);
    const definitionYaml = rewrite(type.definitionYaml, type.id);
    if (icon === type.icon && model3d === type.model3d && definitionYaml === type.definitionYaml) continue;

    type.icon = icon;
    type.model3d = model3d;
    type.definitionYaml = definitionYaml;
    await type.save();
    updated += 1;
    logger.info(`ElementType ${type.id}: fixed asset URLs`, { icon, model3d, definitionYaml });
  }

  logger.info(`ElementType: ${updated} rows fixed`);
  process.exit(0);
}

main().catch((error) => {
  logger.error(`Fix failed: ${error.message}`, { stack: error.stack });
  process.exit(1);
});
