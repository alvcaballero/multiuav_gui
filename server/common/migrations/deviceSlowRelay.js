// Devices.category 'px4_mavros_slow' -> 'px4_mavros' + attributes.slow = true. The slow relay was an
// interface of its own (px4_mavros with its topics renamed); it is a per-device ROS attribute now,
// valid for any ROS interface (models/ros/ros.js buildTelemetryName).
// Runs at startup like the other device migrations. Idempotent: no row matches the second time. The
// JSON is merged in JS, not with json_set, because JSON functions differ per dialect.
const OLD = 'px4_mavros_slow';
const NEW = 'px4_mavros';

export async function migrateDeviceSlowRelay(sequelize, logger) {
  const qi = sequelize.getQueryInterface();
  const q = (name) => qi.quoteIdentifier(name);
  const tables = (await qi.showAllTables()).map((t) => (typeof t === 'string' ? t : t.tableName));
  if (!tables.includes('Devices')) return;

  const [rows] = await sequelize.query(
    `SELECT ${q('id')}, ${q('attributes')} FROM ${q('Devices')} WHERE ${q('category')} = :old`,
    { replacements: { old: OLD } }
  );
  for (const { id, attributes } of rows) {
    const current = (typeof attributes === 'string' ? JSON.parse(attributes) : attributes) ?? {};
    await sequelize.query(
      `UPDATE ${q('Devices')} SET ${q('category')} = :category, ${q('attributes')} = :attributes WHERE ${q('id')} = :id`,
      { replacements: { id, category: NEW, attributes: JSON.stringify({ ...current, slow: true }) } }
    );
  }
  if (rows.length) logger.info(`Migrated ${rows.length} device(s): ${OLD} -> ${NEW} with attributes.slow`);
}
