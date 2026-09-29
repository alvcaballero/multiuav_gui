import { DataTypes } from 'sequelize';
import { LEGACY_ROUTE_ACTION } from '../../models/mission/taskGraph.js';

// Mission/Route -> Mission/Task graph schema. Must run BEFORE sequelize.sync():
// otherwise sync creates an empty MissionTask next to the old table and the rename can
// never happen. Idempotent: every step is guarded on the current on-disk schema.
export async function migrateMissionTaskGraph(sequelize, logger) {
  const qi = sequelize.getQueryInterface();
  const q = (name) => qi.quoteIdentifier(name);
  const tables = (await qi.showAllTables()).map((t) => (typeof t === 'string' ? t : t.tableName));

  if (tables.includes('MissionRoute') && !tables.includes('MissionTask')) {
    // Native rename: SQLite >= 3.26 also rewrites File's REFERENCES to the new name.
    await qi.renameTable('MissionRoute', 'MissionTask');
    tables.push('MissionTask');
    logger.info('Migrated table MissionRoute -> MissionTask');
  }

  if (tables.includes('MissionTask')) {
    const columns = await qi.describeTable('MissionTask');
    if (!columns.taskKey) await qi.addColumn('MissionTask', 'taskKey', { type: DataTypes.STRING });
    if (!columns.dependsOn) await qi.addColumn('MissionTask', 'dependsOn', { type: DataTypes.JSON });
    if (!columns.action) await qi.addColumn('MissionTask', 'action', { type: DataTypes.STRING });

    // Pre-graph rows had no dependencies: each becomes an independent task keyed by its PK.
    await sequelize.query(
      `UPDATE ${q('MissionTask')} SET ${q('taskKey')} = CONCAT('T', ${q('id')}) WHERE ${q('taskKey')} IS NULL`
    );
    await sequelize.query(`UPDATE ${q('MissionTask')} SET ${q('dependsOn')} = '[]' WHERE ${q('dependsOn')} IS NULL`);
    await sequelize.query(`UPDATE ${q('MissionTask')} SET ${q('action')} = :action WHERE ${q('action')} IS NULL`, {
      replacements: { action: LEGACY_ROUTE_ACTION },
    });
  }

  if (tables.includes('File')) await renameColumn(sequelize, logger, 'File', 'routeId', 'taskId');
  // "task" used to mean the ExtApp mission request; it now means a node of the graph.
  if (tables.includes('Mission')) await renameColumn(sequelize, logger, 'Mission', 'task', 'request');
}

async function renameColumn(sequelize, logger, table, from, to) {
  const qi = sequelize.getQueryInterface();
  const columns = await qi.describeTable(table);
  if (!columns[from] || columns[to]) return;
  // Sequelize's SQLite renameColumn rebuilds the table from describeTable and can
  // drop the FK REFERENCES; the native statement keeps them.
  if (sequelize.getDialect() === 'sqlite') {
    const q = (name) => qi.quoteIdentifier(name);
    await sequelize.query(`ALTER TABLE ${q(table)} RENAME COLUMN ${q(from)} TO ${q(to)}`);
  } else {
    await qi.renameColumn(table, from, to);
  }
  logger.info(`Migrated column ${table}.${from} -> ${table}.${to}`);
}
