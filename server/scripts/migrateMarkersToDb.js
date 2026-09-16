// One-off migration: reads server/data/missionConfig.yaml (elements, markersbase,
// assignments) plus the element-type catalogs (config/planning/elementTypes.yaml +
// data/markerTypes.yaml) and populates the new SQL tables (ElementType,
// ElementGroup, ElementItem, Base, Assignment). Idempotent — safe to re-run.
//
// Usage: node server/scripts/migrateMarkersToDb.js

import { readFileSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, resolve } from 'path';
import { parse } from 'yaml';
import sequelize from '../common/sequelize.js';
import { logger } from '../common/logger.js';

const __dirname = dirname(fileURLToPath(import.meta.url));

function readYaml(relativePath) {
  const fullPath = resolve(__dirname, relativePath);
  return parse(readFileSync(fullPath, 'utf8')) || [];
}

async function migrateElementTypes() {
  const staticTypes = readYaml('../config/planning/elementTypes.yaml') || [];
  const customTypes = readYaml('../data/markerTypes.yaml') || [];

  // findOrCreate + backfill: creates rows that don't exist yet, and — for rows
  // that already exist from a previous run — fills in `attributes` if it's
  // still empty (e.g. this script ran before the field existed). A type whose
  // attributes were customized via the admin UI is left untouched.
  let backfilled = 0;
  for (const type of staticTypes) {
    const [elementType, created] = await sequelize.models.ElementType.findOrCreate({
      where: { id: type.id },
      defaults: {
        id: type.id,
        name: type.name,
        description: type.description || '',
        icon: type.icon ?? null,
        model3d: type.model3d ?? null,
        color: type.color ?? null,
        isCustom: false,
        attributes: type.attributes ?? null,
      },
    });
    if (!created && !elementType.attributes && type.attributes) {
      elementType.attributes = type.attributes;
      await elementType.save();
      backfilled += 1;
    }
  }
  for (const type of customTypes) {
    const [elementType, created] = await sequelize.models.ElementType.findOrCreate({
      where: { id: type.id },
      defaults: {
        id: type.id,
        name: type.name,
        description: type.description || '',
        icon: type.icon ?? null,
        model3d: type.model3d ?? null,
        color: type.color ?? null,
        isCustom: true,
        attributes: type.attributes ?? null,
      },
    });
    if (!created && !elementType.attributes && type.attributes) {
      elementType.attributes = type.attributes;
      await elementType.save();
      backfilled += 1;
    }
  }
  logger.info(`Migrated ${staticTypes.length + customTypes.length} element types (${backfilled} backfilled)`);
}

async function migrateElementGroups(missionConfig) {
  const elements = missionConfig.elements || [];
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
  logger.info(`Migrated ${groupCount} element groups, ${itemCount} element items`);
}

async function migrateBases(missionConfig) {
  const markersbase = missionConfig.markersbase || [];
  let count = 0;

  for (const base of markersbase) {
    const [, created] = await sequelize.models.Base.findOrCreate({
      where: { id: base.id },
      defaults: {
        id: base.id,
        typeId: null,
        name: base.name ?? null,
        latitude: base.latitude,
        longitude: base.longitude,
        corners: base.corners ?? null,
      },
    });
    if (created) count += 1;
  }
  logger.info(`Migrated ${count} bases`);
}

async function migrateAssignments(missionConfig) {
  const assignments = missionConfig.assignments || [];
  let count = 0;
  let skipped = 0;

  for (const assignment of assignments) {
    const deviceRef = assignment.device || {};
    const device = await sequelize.models.Device.findOne({
      where: deviceRef.name ? { name: deviceRef.name } : { id: deviceRef.id },
    });
    if (!device) {
      logger.warn(
        `Skipping assignment for base ${assignment.baseId}: device ${JSON.stringify(deviceRef)} not found`
      );
      skipped += 1;
      continue;
    }

    const [, created] = await sequelize.models.Assignment.findOrCreate({
      where: { baseId: assignment.baseId, deviceId: device.id },
      defaults: {
        baseId: assignment.baseId,
        deviceId: device.id,
        settings: assignment.settings || {},
      },
    });
    if (created) count += 1;
  }
  logger.info(`Migrated ${count} assignments (${skipped} skipped, device not found)`);
}

async function main() {
  const missionConfig = readYaml('../data/missionConfig.yaml') || {};

  await migrateElementTypes();
  await migrateElementGroups(missionConfig);
  await migrateBases(missionConfig);
  await migrateAssignments(missionConfig);

  logger.info('Migration complete');
  process.exit(0);
}

main().catch((error) => {
  logger.error(`Migration failed: ${error.message}`, { stack: error.stack });
  process.exit(1);
});
