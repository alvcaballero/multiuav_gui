// One-off: adds "Line C" and "Line D" (windTurbine groups + items) from
// server/data/missionConfig16.yaml into the SQL tables. Same findOrCreate
// pattern as migrateMarkersToDb.js — idempotent, safe to re-run.
//
// Usage: node server/scripts/addLineCD.js

import { readFileSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, resolve } from 'path';
import { parse } from 'yaml';
import sequelize from '../common/sequelize.js';
import { logger } from '../common/logger.js';

const __dirname = dirname(fileURLToPath(import.meta.url));

function readYaml(relativePath) {
  const fullPath = resolve(__dirname, relativePath);
  return parse(readFileSync(fullPath, 'utf8')) || {};
}

async function main() {
  const missionConfig = readYaml('../data/missionConfig16.yaml');
  const elements = (missionConfig.elements || []).filter((g) =>
    ['Line C', 'Line D'].includes((g.name || '').trim())
  );

  let groupCount = 0;
  let itemCount = 0;

  for (const group of elements) {
    const name = (group.name || '').trim();
    const existing = await sequelize.models.ElementGroup.findOne({
      where: { name, typeId: group.type },
    });
    const elementGroup =
      existing ||
      (await sequelize.models.ElementGroup.create({
        typeId: group.type,
        name,
        description: group.description || '',
        linea: group.linea ?? false,
        attributes: {},
      }));
    if (!existing) groupCount += 1;

    for (const item of group.items || []) {
      const [, created] = await sequelize.models.ElementItem.findOrCreate({
        where: { groupId: elementGroup.id, name: item.name },
        defaults: {
          groupId: elementGroup.id,
          name: item.name,
          latitude: item.latitude,
          longitude: item.longitude,
          description: null,
          attributes: null,
        },
      });
      if (created) itemCount += 1;
    }
  }

  logger.info(`Added ${groupCount} element groups, ${itemCount} element items`);
  process.exit(0);
}

main().catch((error) => {
  logger.error(`addLineCD failed: ${error.message}`, { stack: error.stack });
  process.exit(1);
});
