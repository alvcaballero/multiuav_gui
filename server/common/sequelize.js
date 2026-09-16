import { Sequelize, Op } from 'sequelize';

import { useExternalDb, dbType, dbName, dbHost, dbPort, dbUser, dbPassword } from '../config/config.js';
import { setupModels } from '../schemas/database/index.js';
import { logger } from './logger.js';

logger.info('useExternalDb is ' + useExternalDb);

let sequelizeConfig = {
  dialect: 'sqlite',
  storage: 'data/sequelize.sqlite',
  logging: false,
  pool: { max: 1, idle: Infinity, maxUses: Infinity },
};

if (useExternalDb) {
  if (dbType === 'postgres') {
    sequelizeConfig = {
      dialect: 'postgres',
      database: dbName,
      user: dbUser,
      password: dbPassword,
      host: dbHost,
      port: dbPort,
      logging: false,
      dialectOptions: { useUTC: true, timezone: 'UTC' },
    };
  }
  if (dbType === 'mysql') {
    sequelizeConfig = {
      dialect: 'mysql',
      database: dbName,
      user: dbUser,
      password: dbPassword,
      host: dbHost,
      port: dbPort,
      logging: false,
      timezone: '+00:00',
    };
  }
}

const sequelize = new Sequelize(sequelizeConfig);

try {
  await sequelize.authenticate();
  logger.info('DB Connection successfully.');
} catch (error) {
  logger.error('Unable to connect to the database: ', error);
}

setupModels(sequelize);

try {
  await sequelize.sync({ force: false });
  logger.info('DB  models were synchronized successfully.');
} catch (error) {
  logger.error('DB Unable to create tables : ', error);
}

// Additive column migrations — safe to run on every startup.
// Add new ALTER TABLE statements here when adding columns to existing tables.
const migrations = [
  `ALTER TABLE ChatMessage ADD COLUMN hidden INTEGER NOT NULL DEFAULT 0`,
  `ALTER TABLE Devices ADD COLUMN deletedAt DATETIME DEFAULT NULL`,
  // mission-db-schema refactor
  `ALTER TABLE Mission ADD COLUMN planId INTEGER DEFAULT NULL`,
  `ALTER TABLE Mission ADD COLUMN trigger TEXT DEFAULT 'automatic'`,
  `ALTER TABLE MissionPlan ADD COLUMN name TEXT DEFAULT NULL`,
  `ALTER TABLE MissionPlan ADD COLUMN source TEXT NOT NULL DEFAULT 'manual'`,
  `ALTER TABLE Mission ADD COLUMN errorMessage TEXT DEFAULT NULL`,
  // externalId split: the external system's task id no longer lives in the PK.
  `ALTER TABLE Mission ADD COLUMN externalId INTEGER DEFAULT NULL`,
  // route-level error tracking: mirror Mission.errorMessage on each route so a
  // per-UAV failure (load failed, SFTP download empty/unreachable) is persisted.
  `ALTER TABLE MissionRoute ADD COLUMN errorMessage TEXT DEFAULT NULL`,
  // file-level error tracking: a File that failed to download stays in FAIL with
  // no human-readable reason. Persist why (connection lost, download failed, ...).
  `ALTER TABLE File ADD COLUMN errorMessage TEXT DEFAULT NULL`,
  // position history: device-reported timestamp alongside the server-side fixTime
  // (may be missing/inconsistent per payload; kept for later reconciliation).
  `ALTER TABLE PositionHistory ADD COLUMN deviceTime DATETIME DEFAULT NULL`,
  // element type default attributes (geometry defaults inherited by ElementItems).
  `ALTER TABLE ElementTypes ADD COLUMN attributes JSON DEFAULT NULL`,
];

for (const sql of migrations) {
  try {
    await sequelize.query(sql);
  } catch (e) {
    // "duplicate column name" means the column already exists — skip silently
    if (!e.message?.includes('duplicate column')) {
      logger.error(`Migration failed: ${sql}`, e.message);
    }
  }
}

// SQLite has no ALTER TABLE ... ALTER COLUMN / ALTER CONSTRAINT, so column-type
// and FK-reference fixes below rebuild the table (create the corrected shape,
// copy data across, drop the old table, rename). Each is guarded on the stale
// on-disk DDL so it only runs once.
if (sequelize.getDialect() === 'sqlite') {
  // `event.positionId` was declared INTEGER but the app has always stored a
  // [lat, lon, altitude] array in it. SQLite's INTEGER affinity silently stored
  // that as an unparseable comma-joined string (e.g. ",,"), which DataTypes.JSON
  // can't read back. Any existing row whose value isn't valid JSON is reset to
  // NULL since the original coordinates were never recoverable.
  try {
    const [[table]] = await sequelize.query(`SELECT sql FROM sqlite_master WHERE type='table' AND name='event'`);
    if (table?.sql && /`positionId`\s+INTEGER/i.test(table.sql)) {
      await sequelize.transaction(async (t) => {
        await sequelize.query(
          `CREATE TABLE \`event_new\` (
             \`id\` INTEGER PRIMARY KEY AUTOINCREMENT,
             \`type\` VARCHAR(255) NOT NULL,
             \`eventTime\` DATETIME NOT NULL,
             \`deviceId\` INTEGER REFERENCES \`Devices\` (\`id\`),
             \`positionId\` JSON,
             \`missionId\` INTEGER REFERENCES \`Mission\` (\`id\`),
             \`attributes\` JSON
           )`,
          { transaction: t }
        );
        await sequelize.query(
          `INSERT INTO \`event_new\` (id, type, eventTime, deviceId, positionId, missionId, attributes)
           SELECT id, type, eventTime, deviceId,
                  CASE WHEN json_valid(positionId) THEN positionId ELSE NULL END,
                  missionId, attributes
           FROM \`event\``,
          { transaction: t }
        );
        await sequelize.query('DROP TABLE `event`', { transaction: t });
        await sequelize.query('ALTER TABLE `event_new` RENAME TO `event`', { transaction: t });
      });
      logger.info('Migrated event.positionId column from INTEGER to JSON');
    }
  } catch (e) {
    logger.error('Migration failed: event.positionId column type change', e.message);
  }

  // `File.routeId`'s FK was left pointing at the legacy `Route` table (pre-rename
  // to `MissionRoute`), which is now empty/orphaned — so every insert with a
  // non-null routeId permanently fails its FK check and addFile()'s try/catch
  // silently swallows it (files download from the UAV via SFTP but are never
  // recorded in the DB).
  try {
    const [[table]] = await sequelize.query(`SELECT sql FROM sqlite_master WHERE type='table' AND name='File'`);
    if (table?.sql && /REFERENCES\s+`Route`\s*\(/i.test(table.sql)) {
      await sequelize.transaction(async (t) => {
        await sequelize.query(
          `CREATE TABLE \`File_new\` (
             \`id\` INTEGER PRIMARY KEY AUTOINCREMENT,
             \`name\` VARCHAR(255) NOT NULL,
             \`routeId\` INTEGER REFERENCES \`MissionRoute\` (\`id\`),
             \`missionId\` INTEGER REFERENCES \`Mission\` (\`id\`),
             \`deviceId\` INTEGER REFERENCES \`Devices\` (\`id\`),
             \`status\` INTEGER NOT NULL DEFAULT 0,
             \`type\` VARCHAR(255),
             \`path\` VARCHAR(255),
             \`path2\` VARCHAR(255),
             \`source\` JSON,
             \`date\` DATETIME NOT NULL,
             \`attributes\` JSON,
             errorMessage TEXT DEFAULT NULL
           )`,
          { transaction: t }
        );
        await sequelize.query(
          `INSERT INTO \`File_new\` (id, name, routeId, missionId, deviceId, status, type, path, path2, source, date, attributes, errorMessage)
           SELECT id, name, routeId, missionId, deviceId, status, type, path, path2, source, date, attributes, errorMessage
           FROM \`File\``,
          { transaction: t }
        );
        await sequelize.query('DROP TABLE `File`', { transaction: t });
        await sequelize.query('ALTER TABLE `File_new` RENAME TO `File`', { transaction: t });
      });
      logger.info('Migrated File.routeId foreign key from legacy Route table to MissionRoute');
    }
  } catch (e) {
    logger.error('Migration failed: File.routeId FK reference change', e.message);
  }

  // `Bases.id` was a client-generated STRING (`base_N`), not guaranteed unique
  // across independently-created bases (the client fell back to
  // `base_${arrayIndex}` for a base with no id yet, so two different bases
  // could collide on the same id and silently overwrite each other via
  // upsert). Rebuild `Bases` with a real INTEGER AUTOINCREMENT primary key
  // assigned by the server. `Assignments.baseId` (the only other table with a
  // real reference to Bases) is cleared as part of this migration rather than
  // remapped — existing assignments are not considered valuable data to
  // preserve, and re-assigning a device to a base is a one-click action in
  // the UI.
  try {
    const [[table]] = await sequelize.query(`SELECT sql FROM sqlite_master WHERE type='table' AND name='Bases'`);
    if (table?.sql && /`id`\s+VARCHAR/i.test(table.sql)) {
      await sequelize.transaction(async (t) => {
        // Assignments.baseId REFERENCES Bases(id) — SQLite enforces that FK
        // even mid-DROP, so rebuilding Bases while Assignments still points
        // at it fails with SQLITE_CONSTRAINT. `foreign_keys` itself is a
        // no-op once a transaction has started (and this driver re-enables
        // it on every BEGIN regardless), but `defer_foreign_keys` IS
        // settable mid-transaction and postpones the check to COMMIT — by
        // then both Bases and Assignments have been rebuilt below, so the
        // constraint holds again and the commit succeeds.
        await sequelize.query('PRAGMA defer_foreign_keys = ON', { transaction: t });

        const oldBases = await sequelize.query(
          `SELECT id, typeId, name, latitude, longitude, corners FROM \`Bases\``,
          { transaction: t, type: sequelize.QueryTypes.SELECT }
        );

        // Preserve the existing `base_N` ordering as the new integer id when
        // possible; anything that doesn't match the pattern gets a fresh id
        // continuing after the highest one seen, so every row still gets a
        // unique id with no exceptions.
        const idMap = new Map(); // oldId (string) -> newId (number)
        let nextFreeId = 1;
        for (const base of oldBases) {
          const match = /^base_(\d+)$/.exec(base.id);
          if (match) {
            const candidate = Number(match[1]) + 1; // avoid 0 out of caution
            idMap.set(base.id, candidate);
            nextFreeId = Math.max(nextFreeId, candidate + 1);
          }
        }
        for (const base of oldBases) {
          if (!idMap.has(base.id)) {
            idMap.set(base.id, nextFreeId++);
          }
        }

        await sequelize.query(
          `CREATE TABLE \`Bases_new\` (
             \`id\` INTEGER PRIMARY KEY AUTOINCREMENT,
             \`typeId\` VARCHAR(255) REFERENCES \`ElementTypes\` (\`id\`),
             \`name\` VARCHAR(255),
             \`latitude\` FLOAT,
             \`longitude\` FLOAT,
             \`corners\` JSON
           )`,
          { transaction: t }
        );
        for (const base of oldBases) {
          await sequelize.query(
            `INSERT INTO \`Bases_new\` (id, typeId, name, latitude, longitude, corners)
             VALUES (:id, :typeId, :name, :latitude, :longitude, :corners)`,
            {
              transaction: t,
              replacements: {
                id: idMap.get(base.id),
                typeId: base.typeId ?? null,
                name: base.name ?? null,
                latitude: base.latitude,
                longitude: base.longitude,
                corners: base.corners ?? null,
              },
            }
          );
        }
        await sequelize.query('DROP TABLE `Bases`', { transaction: t });
        await sequelize.query('ALTER TABLE `Bases_new` RENAME TO `Bases`', { transaction: t });

        // Assignments.baseId can't be meaningfully remapped without knowing
        // which new base a client actually wants (its old string id carries
        // no relation to the new integer ids beyond this migration's own
        // renumbering). Rebuilt empty with the corrected column type —
        // existing rows are intentionally not preserved (confirmed
        // acceptable: this data isn't considered valuable).
        const [oldCount] = await sequelize.query(`SELECT COUNT(*) as count FROM \`Assignments\``, {
          transaction: t,
          type: sequelize.QueryTypes.SELECT,
        });
        await sequelize.query(
          `CREATE TABLE \`Assignments_new\` (
             \`id\` INTEGER PRIMARY KEY AUTOINCREMENT,
             \`baseId\` INTEGER NOT NULL REFERENCES \`Bases\` (\`id\`),
             \`deviceId\` INTEGER NOT NULL REFERENCES \`Devices\` (\`id\`),
             \`settings\` JSON DEFAULT '{}'
           )`,
          { transaction: t }
        );
        await sequelize.query('DROP TABLE `Assignments`', { transaction: t });
        await sequelize.query('ALTER TABLE `Assignments_new` RENAME TO `Assignments`', { transaction: t });

        logger.info(
          `Migrated Bases (${oldBases.length} rows) id column from VARCHAR to INTEGER AUTOINCREMENT; ` +
            `rebuilt Assignments with the corrected column type (${oldCount.count} pre-existing rows dropped)`
        );
      });
    }
  } catch (e) {
    logger.error('Migration failed: Bases.id column type change (VARCHAR -> INTEGER autoincrement)', e.message);
  }
}

// Data backfill (idempotent via WHERE externalId IS NULL): historical automatic
// missions stored the external task id AS their primary key. Copy it into the new
// externalId column so ExtApp callbacks keep addressing them by the external id.
// Manual missions (trigger='manual') have no external origin and stay NULL.
try {
  await sequelize.query(`UPDATE Mission SET externalId = id WHERE trigger = 'automatic' AND externalId IS NULL`);
} catch (e) {
  logger.error('Migration failed: backfill Mission.externalId', e.message);
}

export default sequelize;
export { Op };
