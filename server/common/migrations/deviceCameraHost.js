// Devices.ip -> Devices.camera[].host, and Devices.user / Devices.pwd / Devices.model gone. The address was only ever
// used to reach a camera's stream, the file-download credentials live in `files[].url` now, and
// `model` was never read or written by anything.
// Must run BEFORE sequelize.sync() like the other schema migrations and is idempotent: every step is
// guarded on the current on-disk schema, so a fresh database (no Devices table yet) is a no-op.
const LEGACY_COLUMNS = ['ip', 'user', 'pwd', 'model'];

export async function migrateDeviceCameraHost(sequelize, logger) {
  const qi = sequelize.getQueryInterface();
  const q = (name) => qi.quoteIdentifier(name);
  const tables = (await qi.showAllTables()).map((t) => (typeof t === 'string' ? t : t.tableName));
  if (!tables.includes('Devices')) return;

  const columns = await qi.describeTable('Devices');
  if (columns.ip && columns.camera) {
    const [rows] = await sequelize.query(
      `SELECT ${q('id')}, ${q('ip')}, ${q('camera')} FROM ${q('Devices')} WHERE ${q('ip')} IS NOT NULL AND ${q('camera')} IS NOT NULL`
    );
    for (const { id, ip, camera } of rows) {
      const cameras = typeof camera === 'string' ? JSON.parse(camera) : camera;
      if (!Array.isArray(cameras)) continue;
      // A Websocket camera has no stream address; a host the camera already carries wins.
      const next = cameras.map((cam) => (cam.type === 'Websocket' || cam.host ? cam : { ...cam, host: ip }));
      await sequelize.query(`UPDATE ${q('Devices')} SET ${q('camera')} = :camera WHERE ${q('id')} = :id`, {
        replacements: { id, camera: JSON.stringify(next) },
      });
    }
  }

  // Native DROP COLUMN: Sequelize's SQLite removeColumn rebuilds the table and can drop FK REFERENCES.
  const dropped = LEGACY_COLUMNS.filter((name) => columns[name]);
  for (const name of dropped) await sequelize.query(`ALTER TABLE ${q('Devices')} DROP COLUMN ${q(name)}`);
  if (dropped.length) logger.info(`Migrated Devices: ${dropped.join(', ')} dropped (ip moved to camera[].host)`);
}
