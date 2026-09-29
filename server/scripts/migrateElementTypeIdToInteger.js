// One-off migration: ElementType.id goes from a hand-typed STRING PK
// ('windTurbine', 'goliathCrane', 'custom_...') to an autoincrement INTEGER,
// matching every other table in this DB. SQLite has no ALTER COLUMN TYPE, so
// this rebuilds all three affected tables (ElementTypes, ElementGroups,
// Bases) under one transaction: create a _new table with the right schema,
// copy rows across (ElementGroups/Bases' `typeId` remapped via the
// old-string -> new-integer table built while copying ElementTypes), drop
// the old table, rename _new into place. Also renames each type's asset
// folder (server/data/element-types/<oldId>/ -> <newId>/) to match, OUTSIDE
// the DB transaction (filesystem has no rollback) — run only after the DB
// side committed successfully.
//
// Usage: node server/scripts/migrateElementTypeIdToInteger.js

import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import sequelize from '../common/sequelize.js';
import { logger } from '../common/logger.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ASSETS_DIR = path.resolve(__dirname, '../data/element-types');

async function migrate() {
  const oldTypes = await sequelize.query('SELECT * FROM ElementTypes ORDER BY id', {
    type: sequelize.QueryTypes.SELECT,
  });
  if (oldTypes.length === 0) {
    logger.info('No ElementTypes rows — nothing to migrate.');
    return { idMap: new Map() };
  }
  // Old id (string) -> new id (integer), assigned in a deterministic order
  // so a re-run against a freshly-copied DB produces the same mapping.
  const idMap = new Map(oldTypes.map((row, index) => [row.id, index + 1]));

  // SQLite refuses to DROP a table another table still declares a FK
  // REFERENCES against, even mid-rebuild — foreign_keys can't be toggled
  // inside a transaction either. sequelize.transaction() opens its own
  // connection/BEGIN under the hood, which silently dropped this pragma
  // (verified: a plain sequelize.query() sequence keeps it, .transaction()
  // doesn't) — so this uses a hand-rolled BEGIN/COMMIT/ROLLBACK on the same
  // query() connection the pragma was set on, instead. Restored to ON (and
  // checked) right after, per SQLite's own recommended rebuild recipe
  // (https://www.sqlite.org/lang_altertable.html#otheralter).
  await sequelize.query('PRAGMA foreign_keys = OFF');
  await sequelize.query('BEGIN TRANSACTION');
  try {
    // ElementTypes must be fully swapped in BEFORE ElementGroups_new/
    // Bases_new are created — their `typeId REFERENCES ElementTypes(id)` is
    // checked against whatever table is named ElementTypes at INSERT time,
    // so it has to already be the new, integer-keyed one.
    await sequelize.query(
      `CREATE TABLE ElementTypes_new (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name VARCHAR(255) NOT NULL,
        description VARCHAR(255) DEFAULT '',
        icon VARCHAR(255),
        model3d VARCHAR(255),
        color VARCHAR(255),
        isCustom TINYINT(1) DEFAULT 0,
        attributes JSON DEFAULT NULL,
        definitionYaml TEXT DEFAULT NULL
      )`
    );
    for (const row of oldTypes) {
      const newId = idMap.get(row.id);
      // icon/model3d/definitionYaml are stored as literal URLs
      // (`/api/markers/types/<id>/icon`, written by uploadIcon et al.), not
      // derived at read time — the old string id is baked into each one and
      // has to be swapped for the new integer id here, or every asset link
      // 404s the moment the old id's folder gets renamed below.
      const rewriteAssetUrl = (url) => (url ? url.replace(`/types/${row.id}/`, `/types/${newId}/`) : url);
      await sequelize.query(
        `INSERT INTO ElementTypes_new
          (id, name, description, icon, model3d, color, isCustom, attributes, definitionYaml)
         VALUES (:id, :name, :description, :icon, :model3d, :color, :isCustom, :attributes, :definitionYaml)`,
        {
          replacements: {
            ...row,
            id: newId,
            icon: rewriteAssetUrl(row.icon),
            model3d: rewriteAssetUrl(row.model3d),
            definitionYaml: rewriteAssetUrl(row.definitionYaml),
          },
        }
      );
    }
    await sequelize.query('DROP TABLE ElementTypes');
    await sequelize.query('ALTER TABLE ElementTypes_new RENAME TO ElementTypes');

    const oldGroups = await sequelize.query('SELECT * FROM ElementGroups', {
      type: sequelize.QueryTypes.SELECT,
    });
    await sequelize.query(
      `CREATE TABLE ElementGroups_new (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        typeId INTEGER NOT NULL REFERENCES ElementTypes (id) ON DELETE NO ACTION ON UPDATE CASCADE,
        name VARCHAR(255) NOT NULL,
        description VARCHAR(255) DEFAULT '',
        linea TINYINT(1) DEFAULT 0,
        attributes JSON DEFAULT '{}',
        deletedAt DATETIME DEFAULT NULL
      )`
    );
    for (const row of oldGroups) {
      const newTypeId = idMap.get(row.typeId);
      if (newTypeId === undefined) {
        throw new Error(`ElementGroup ${row.id} references unknown typeId "${row.typeId}" — aborting migration`);
      }
      await sequelize.query(
        `INSERT INTO ElementGroups_new (id, typeId, name, description, linea, attributes, deletedAt)
         VALUES (:id, :typeId, :name, :description, :linea, :attributes, :deletedAt)`,
        { replacements: { ...row, typeId: newTypeId } }
      );
    }
    await sequelize.query('DROP TABLE ElementGroups');
    await sequelize.query('ALTER TABLE ElementGroups_new RENAME TO ElementGroups');

    const oldBases = await sequelize.query('SELECT * FROM Bases', { type: sequelize.QueryTypes.SELECT });
    await sequelize.query(
      `CREATE TABLE Bases_new (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        typeId INTEGER REFERENCES ElementTypes (id),
        name VARCHAR(255),
        latitude FLOAT,
        longitude FLOAT,
        corners JSON,
        altitude FLOAT DEFAULT NULL,
        azimFront FLOAT DEFAULT NULL,
        attributes JSON DEFAULT NULL
      )`
    );
    for (const row of oldBases) {
      // typeId is nullable on Base — a base with no type stays untyped.
      const newTypeId = row.typeId == null ? null : idMap.get(row.typeId);
      if (row.typeId != null && newTypeId === undefined) {
        throw new Error(`Base ${row.id} references unknown typeId "${row.typeId}" — aborting migration`);
      }
      await sequelize.query(
        `INSERT INTO Bases_new
          (id, typeId, name, latitude, longitude, corners, altitude, azimFront, attributes)
         VALUES (:id, :typeId, :name, :latitude, :longitude, :corners, :altitude, :azimFront, :attributes)`,
        { replacements: { ...row, typeId: newTypeId } }
      );
    }
    await sequelize.query('DROP TABLE Bases');
    await sequelize.query('ALTER TABLE Bases_new RENAME TO Bases');

    await sequelize.query('COMMIT');
  } catch (error) {
    await sequelize.query('ROLLBACK');
    throw error;
  } finally {
    await sequelize.query('PRAGMA foreign_keys = ON');
  }

  const violations = await sequelize.query('PRAGMA foreign_key_check', { type: sequelize.QueryTypes.SELECT });
  if (violations.length > 0) {
    throw new Error(`Post-migration foreign_key_check found violations: ${JSON.stringify(violations)}`);
  }

  logger.info('DB side of the migration committed.', { idMap: Object.fromEntries(idMap) });

  // Filesystem side — no transaction/rollback here, run only after the DB
  // commit above succeeded. Renaming into a numeric-only name can't collide
  // with an existing string-named folder, so plain fs.renameSync is safe.
  for (const [oldId, newId] of idMap) {
    const oldDir = path.join(ASSETS_DIR, String(oldId));
    const newDir = path.join(ASSETS_DIR, String(newId));
    if (fs.existsSync(oldDir)) {
      fs.renameSync(oldDir, newDir);
      logger.info(`Renamed asset folder ${oldId} -> ${newId}`);
    }
  }

  return { idMap };
}

migrate()
  .then(({ idMap }) => {
    logger.info(`Migration complete. ${idMap.size} ElementType rows remapped.`);
    process.exit(0);
  })
  .catch((error) => {
    logger.error(`Migration failed: ${error.message}`, { stack: error.stack });
    process.exit(1);
  });
