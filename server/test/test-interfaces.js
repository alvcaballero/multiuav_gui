import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, writeFileSync, rmSync, readFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { interfacesDir } from '../config/config.js';
import { buildCatalog, loadInterfaces } from '../models/interfaces/interfaceCatalog.js';
import { readInterfaceDir, writeInterfaceFile } from '../models/interfaces/interfaceStore.js';

// config/devices/interfaces/: one file per interface, validated as a whole. The golden is what the
// old devices_msg.yaml resolved to (minus griffin.speed, whose `name: ' '` never subscribed
// anything): splitting the file must not change one value.
const golden = JSON.parse(readFileSync(new URL('./api/json/interfaces_golden.json', import.meta.url)));

const withDir = (files, fn) => {
  const dir = mkdtempSync(path.join(tmpdir(), 'interfaces-'));
  try {
    for (const [name, content] of Object.entries(files)) writeFileSync(path.join(dir, name), content);
    return fn(dir);
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
};

const ros = (extra = {}) => ({
  subscribers: { position: { name: '/gps', messageType: 'sensor_msgs/NavSatFix', ...extra } },
});
const problemsOf = (def) => buildCatalog({ x: def }).problems.map(({ path: p, message }) => [p.join('.'), message]);

describe('the real interface files', () => {
  const { catalog, problems } = loadInterfaces(interfacesDir);

  test('every file validates', () => {
    assert.deepEqual(problems, []);
  });

  test('resolve to exactly what devices_msg.yaml did', () => {
    assert.deepEqual(catalog, golden);
  });
});

describe('readInterfaceDir', () => {
  test('the file name is the key, and only *.yaml are read', () => {
    withDir({ 'a_1.yaml': 'subscribers: {}\n', 'a_1.yaml.bak': 'x: [', 'notes.txt': 'hi' }, (dir) => {
      const { raw, problems } = readInterfaceDir(dir);
      assert.deepEqual(raw, { a_1: { subscribers: {} } });
      assert.deepEqual(problems, []);
    });
  });

  test('a file that is not named after a key is reported, not ignored', () => {
    withDir({ 'dji M210 copy.yaml': 'subscribers: {}\n' }, (dir) => {
      const { raw, problems } = readInterfaceDir(dir);
      assert.deepEqual(raw, {});
      assert.equal(problems.length, 1);
      assert.match(problems[0].message, /not named after a valid interface key/);
    });
  });

  test('a file YAML cannot parse is reported with its line, and the others still load', () => {
    withDir({ 'good.yaml': 'subscribers: {}\n', 'bad.yaml': 'subscribers:\n  position: [\n' }, (dir) => {
      const { raw, problems } = readInterfaceDir(dir);
      assert.deepEqual(Object.keys(raw), ['good']);
      assert.deepEqual(problems[0].path, ['bad']);
      assert.match(problems[0].message, /bad\.yaml: .*line \d+/);
    });
  });

  test('a duplicated key inside a file is a parse error', () => {
    withDir({ 'dup.yaml': 'subscribers: {}\nsubscribers: {}\n' }, (dir) => {
      assert.equal(readInterfaceDir(dir).problems.length, 1);
    });
  });

  test('writeInterfaceFile refuses a key that could leave the folder', async () => {
    await withDir({}, async (dir) => {
      await assert.rejects(writeInterfaceFile(dir, '../escape', {}), /not a valid interface key/);
    });
  });
});

describe('buildCatalog', () => {
  test('a well formed interface loads', () => {
    assert.deepEqual(buildCatalog({ x: ros({ priority: 10, fresh_ms: 3000, watchdog: false }) }).catalog.x, ros({ priority: 10, fresh_ms: 3000, watchdog: false }));
  });

  test('a typo in a field is an error, not a silent no-op', () => {
    assert.deepEqual(problemsOf({ subscribers: { position: { nmae: '/gps', messageType: 'sensor_msgs/NavSatFix' } } }), [
      ['x.subscribers.position', "Unrecognized key(s) in object: 'nmae'"],
    ]);
    assert.deepEqual(problemsOf({ subscriber: {} }), [['x', "Unrecognized key(s) in object: 'subscriber'"]]);
  });

  test('a param of the wrong type is an error', () => {
    assert.equal(problemsOf(ros({ rate_hz: 'two' })).length, 1);
    assert.equal(problemsOf(ros({ unit: 'grad' })).length, 1);
  });

  test('a ROS name must start with / and have no spaces (griffin.speed had " ")', () => {
    assert.deepEqual(problemsOf({ subscribers: { speed: { name: ' ', messageType: 'geometry_msgs/TwistStamped' } } }), [
      ['x.subscribers.speed.name', "must start with '/' and contain no spaces"],
    ]);
  });

  test('every entry names its messageType, and a ROS entry its name', () => {
    assert.deepEqual(problemsOf({ services: { command_mission: { name: '/start' } } }), [
      ['x.services.command_mission', 'missing messageType'],
    ]);
    assert.deepEqual(problemsOf({ services: { command_mission: { messageType: 'std_srvs/SetBool' } } }), [
      ['x.services.command_mission', 'missing name (the ROS topic/service/action)'],
    ]);
  });

  test('a MAVLink interface has no names and does not mix in ROS messages', () => {
    assert.deepEqual(problemsOf({ subscribers: { battery: { messageType: 'mavlink/BATTERY_STATUS' } } }), []);
    assert.deepEqual(problemsOf({ subscribers: { battery: { name: '/b', messageType: 'mavlink/BATTERY_STATUS' } } }), [
      ['x.subscribers.battery', 'a MAVLink entry has no name (there are no topics)'],
    ]);
    assert.deepEqual(
      problemsOf({
        subscribers: { battery: { messageType: 'mavlink/BATTERY_STATUS' }, position: { name: '/gps', messageType: 'sensor_msgs/NavSatFix' } },
      }),
      [['x.subscribers.position', "mixes message formats: 'sensor_msgs/NavSatFix' in a mavlink interface"]]
    );
  });

  test('an invalid interface is left out and the others still load', () => {
    const { catalog, problems } = buildCatalog({ good: ros(), bad: { subscribers: { position: { nmae: '/x' } } } });
    assert.deepEqual(Object.keys(catalog), ['good']);
    assert.ok(problems.every((p) => p.path[0] === 'bad'));
  });
});
