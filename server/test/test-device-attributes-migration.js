import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { Sequelize } from 'sequelize';
import { migrateDeviceAttributes } from '../common/migrations/deviceAttributes.js';

const logger = { info() {}, warn() {}, error() {} };
const memory = () => new Sequelize({ dialect: 'sqlite', storage: ':memory:', logging: false });

describe('Devices.systemId -> Devices.attributes', () => {
  test('moves the system id into attributes and drops the column; a second run changes nothing', async () => {
    const db = memory();
    await db.query('CREATE TABLE `Devices` (`id` INTEGER PRIMARY KEY, `name` TEXT, `systemId` INTEGER)');
    await db.query("INSERT INTO `Devices` (id, name, systemId) VALUES (1, 'mav', 7), (2, 'ros', NULL)");

    await migrateDeviceAttributes(db, logger);
    await migrateDeviceAttributes(db, logger);

    const columns = await db.getQueryInterface().describeTable('Devices');
    assert.equal(columns.systemId, undefined);
    assert.ok(columns.attributes);
    const [rows] = await db.query('SELECT id, attributes FROM `Devices` ORDER BY id');
    assert.deepEqual(JSON.parse(rows[0].attributes), { systemId: 7 });
    assert.equal(rows[1].attributes, null);
    await db.close();
  });

  test('a database with no Devices table yet is left alone', async () => {
    const db = memory();
    await migrateDeviceAttributes(db, logger);
    assert.deepEqual(await db.getQueryInterface().showAllTables(), []);
    await db.close();
  });
});
