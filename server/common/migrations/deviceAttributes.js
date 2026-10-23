import { DataTypes } from 'sequelize';

// Devices.systemId -> Devices.attributes.systemId. `attributes` holds the settings specific to each device that
// depend on something else about it (schemas/zod/devices.js, ATTRIBUTE_GROUPS). Must run
// BEFORE sequelize.sync() like the other schema migrations, and is idempotent: every step is guarded
// on the current on-disk schema, so a fresh database (no Devices table yet) is a no-op.
export async function migrateDeviceAttributes(sequelize, logger) {
  const qi = sequelize.getQueryInterface();
  const q = (name) => qi.quoteIdentifier(name);
  const tables = (await qi.showAllTables()).map((t) => (typeof t === 'string' ? t : t.tableName));
  if (!tables.includes('Devices')) return;

  let columns = await qi.describeTable('Devices');
  if (!columns.attributes) {
    await qi.addColumn('Devices', 'attributes', { type: DataTypes.JSON, allowNull: true });
    columns = await qi.describeTable('Devices');
  }
  if (!columns.systemId) return;

  const [rows] = await sequelize.query(
    `SELECT ${q('id')}, ${q('systemId')} FROM ${q('Devices')} WHERE ${q('systemId')} IS NOT NULL AND ${q('attributes')} IS NULL`
  );
  for (const { id, systemId } of rows) {
    await sequelize.query(`UPDATE ${q('Devices')} SET ${q('attributes')} = :attributes WHERE ${q('id')} = :id`, {
      replacements: { id, attributes: JSON.stringify({ systemId }) },
    });
  }
  // Native DROP COLUMN: Sequelize's SQLite removeColumn rebuilds the table and can drop FK REFERENCES.
  await sequelize.query(`ALTER TABLE ${q('Devices')} DROP COLUMN ${q('systemId')}`);
  logger.info(`Migrated Devices.systemId -> Devices.attributes (${rows.length} device(s))`);
}
