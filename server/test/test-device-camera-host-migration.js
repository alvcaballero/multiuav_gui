import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { Sequelize } from 'sequelize';
import { migrateDeviceCameraHost } from '../common/migrations/deviceCameraHost.js';

const logger = { info() {}, warn() {}, error() {} };
const memory = () => new Sequelize({ dialect: 'sqlite', storage: ':memory:', logging: false });

describe('Devices.ip -> camera[].host', () => {
  test('copies the ip into every stream camera, drops ip/user/pwd/model; a second run changes nothing', async () => {
    const db = memory();
    await db.query('CREATE TABLE `Devices` (`id` INTEGER PRIMARY KEY, `name` TEXT, `ip` TEXT, `user` TEXT, `pwd` TEXT, `model` TEXT, `camera` JSON)');
    const cameras = JSON.stringify([
      { type: 'WebRTC', source: 'main' },
      { type: 'Websocket', source: 'ros' },
      { type: 'RTSP', source: 'thermal', host: '10.0.0.9' },
    ]);
    await db.query('INSERT INTO `Devices` (id, name, ip, user, pwd, camera) VALUES (1, "a", "10.0.0.5", "u", "p", :cameras), (2, "b", "10.0.0.6", NULL, NULL, NULL)', {
      replacements: { cameras },
    });

    await migrateDeviceCameraHost(db, logger);
    await migrateDeviceCameraHost(db, logger);

    const columns = await db.getQueryInterface().describeTable('Devices');
    for (const gone of ['ip', 'user', 'pwd', 'model']) assert.equal(columns[gone], undefined, gone);
    const [rows] = await db.query('SELECT id, camera FROM `Devices` ORDER BY id');
    assert.deepEqual(JSON.parse(rows[0].camera), [
      { type: 'WebRTC', source: 'main', host: '10.0.0.5' },
      { type: 'Websocket', source: 'ros' },
      { type: 'RTSP', source: 'thermal', host: '10.0.0.9' },
    ]);
    assert.equal(rows[1].camera, null);
    await db.close();
  });

  test('a database with no Devices table yet is left alone', async () => {
    const db = memory();
    await migrateDeviceCameraHost(db, logger);
    assert.deepEqual(await db.getQueryInterface().showAllTables(), []);
    await db.close();
  });
});
