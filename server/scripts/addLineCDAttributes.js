// One-off: sets `attributes` (geometry) on the ElementItems added by
// addLineCD.js (Line C / Line D — groups 9 and 10). Idempotent — safe to
// re-run, always overwrites to the given value.
//
// Usage: node server/scripts/addLineCDAttributes.js

import sequelize from '../common/sequelize.js';
import { logger } from '../common/logger.js';

const GEOMETRY_ATTRIBUTES = {
  geometry: {
    geometry_type: 'circle',
    dimensions: { radius: 28, height: 108 },
    yaw: 0,
  },
};

async function main() {
  const groups = await sequelize.models.ElementGroup.findAll({
    where: { name: ['Line C', 'Line D'] },
  });
  const groupIds = groups.map((g) => g.id);

  const [count] = await sequelize.models.ElementItem.update(
    { attributes: GEOMETRY_ATTRIBUTES },
    { where: { groupId: groupIds } }
  );

  logger.info(`Set geometry attributes on ${count} element items (groups ${groupIds.join(', ')})`);
  process.exit(0);
}

main().catch((error) => {
  logger.error(`addLineCDAttributes failed: ${error.message}`, { stack: error.stack });
  process.exit(1);
});
