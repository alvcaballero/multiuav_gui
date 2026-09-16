// One-off: adds "Elements" groups (building_tall, building_short, crane,
// towerlight) with their items. Same findOrCreate pattern as
// migrateMarkersToDb.js / addLineCD.js — idempotent, safe to re-run.
//
// Usage: node server/scripts/addBuildingsElements.js

import sequelize from '../common/sequelize.js';
import { logger } from '../common/logger.js';

const GROUPS = [
  {
    type: 'building_tall',
    name: 'Elements',
    linea: true,
    description: 'rectanguar buildings x=35m y=20m y de altura 46m',
    items: [
      { latitude: 37.410553536507805, longitude: -6.037373874645823, name: 'building 1', heading: 180 },
      { latitude: 37.4097969189515, longitude: -6.037369464687515, name: 'building 3', heading: 0 },
    ],
    geometry: { geometry_type: 'rectangle', dimensions: { width: 35, length: 20, height: 46 } },
  },
  {
    type: 'building_short',
    name: 'Elements',
    linea: true,
    description: 'rectanguar buildings x=35m y=20m y de altura 30m',
    items: [{ latitude: 37.41016121724752, longitude: -6.037942759174626, name: 'building 2', heading: 90 }],
    geometry: { geometry_type: 'rectangle', dimensions: { width: 35, length: 20, height: 30 } },
  },
  {
    type: 'crane',
    name: 'Elements',
    linea: true,
    description: 'tower altitude: 60m',
    items: [{ latitude: 37.410190462469544, longitude: -6.0373616968579995, heading: 290 }],
    geometry: { geometry_type: 'circle', dimensions: { radius: 3, height: 60 } },
  },
  {
    type: 'towerlight',
    name: 'Elements',
    linea: true,
    description: undefined,
    items: [
      { latitude: 37.41068715470104, longitude: -6.036803969670018 },
      { latitude: 37.41044876299871, longitude: -6.036398793082924 },
      { latitude: 37.410022589850485, longitude: -6.036542372913544 },
      { latitude: 37.40974929391673, longitude: -6.036916271417198 },
      { latitude: 37.40939971951394, longitude: -6.037382429722442 },
      { latitude: 37.409479365632606, longitude: -6.037970614384335 },
      { latitude: 37.40977048628335, longitude: -6.0383295639604455 },
      { latitude: 37.41018765720281, longitude: -6.038212433045771 },
      { latitude: 37.4107519143907, longitude: -6.038042404299233 },
      { latitude: 37.4107519143907, longitude: -6.037461131763763 },
    ],
    geometry: { geometry_type: 'circle', dimensions: { radius: 3, height: 30 } },
  },
];

async function main() {
  let groupCount = 0;
  let itemCount = 0;
  let itemIndex = 0;

  for (const group of GROUPS) {
    const existing = await sequelize.models.ElementGroup.findOne({
      where: { name: group.name, typeId: group.type },
    });
    const elementGroup =
      existing ||
      (await sequelize.models.ElementGroup.create({
        typeId: group.type,
        name: group.name,
        description: group.description || '',
        linea: group.linea ?? false,
        attributes: {},
      }));
    if (!existing) groupCount += 1;

    for (const item of group.items) {
      itemIndex += 1;
      const name = item.name || `${group.type} ${itemIndex}`;
      const attributes = {
        geometry: {
          ...group.geometry,
          ...(item.heading !== undefined ? { yaw: item.heading } : {}),
        },
      };

      const [, created] = await sequelize.models.ElementItem.findOrCreate({
        where: { groupId: elementGroup.id, name },
        defaults: {
          groupId: elementGroup.id,
          name,
          latitude: item.latitude,
          longitude: item.longitude,
          description: null,
          attributes,
        },
      });
      if (created) itemCount += 1;
    }
  }

  logger.info(`Added ${groupCount} element groups, ${itemCount} element items`);
  process.exit(0);
}

main().catch((error) => {
  logger.error(`addBuildingsElements failed: ${error.message}`, { stack: error.stack });
  process.exit(1);
});
