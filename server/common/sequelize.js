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
