import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, writeFileSync, rmSync, readFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { interfacesDir } from '../config/config.js';
import { buildCatalog, loadInterfaces, mergeInterface, rootOf } from '../models/interfaces/interfaceCatalog.js';
import { readInterfaceDir, writeInterfaceFile } from '../models/interfaces/interfaceStore.js';
import { categoryModel } from '../models/category.js';

// config/devices/interfaces/: one file per interface, validated as a whole. The golden is what the
// old devices_msg.yaml resolved to (minus griffin.speed, whose `name: ' '` never subscribed
// anything): splitting the file, and rewriting variants with `extends`, must not change one
// endpoint. The identity fields F1 added (and `mission:`, which replaced the per-category
// override list of missionSymbols.js) are compared apart.
const golden = JSON.parse(readFileSync(new URL('./api/json/interfaces_golden.json', import.meta.url)));
const IDENTITY = ['extends', 'firmwares', 'vehicleTypes', 'control', 'defaultModel', 'mission'];
const endpointsOnly = (def) => Object.fromEntries(Object.entries(def).filter(([key]) => !IDENTITY.includes(key)));

const withDir = (files, fn) => {
  const dir = mkdtempSync(path.join(tmpdir(), 'interfaces-'));
  try {
    for (const [name, content] of Object.entries(files)) writeFileSync(path.join(dir, name), content);
    return fn(dir);
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
};

const IDENTITY_FIELDS = { firmwares: ['px4'], vehicleTypes: ['multirotor'] };
const ros = (extra = {}) => ({
  ...IDENTITY_FIELDS,
  subscribers: { position: { name: '/gps', messageType: 'sensor_msgs/NavSatFix', ...extra } },
});
const problemsOf = (def) => buildCatalog({ x: def }).problems.map(({ path: p, message }) => [p.join('.'), message]);

describe('the real interface files', () => {
  const { catalog, problems } = loadInterfaces(interfacesDir);

  test('every file validates', () => {
    assert.deepEqual(problems, []);
  });

  test('resolve to exactly the endpoints devices_msg.yaml had', () => {
    assert.deepEqual(Object.keys(catalog).sort(), Object.keys(golden).sort());
    for (const [key, def] of Object.entries(catalog)) assert.deepEqual(endpointsOnly(def), golden[key], key);
  });

  test('dji_M210_melodic is the noetic stack with its own battery message', () => {
    assert.equal(catalog.dji_M210_melodic.extends, 'dji_M210_noetic');
    assert.equal(catalog.dji_M210_melodic.subscribers.battery.messageType, 'aerialcore_common/BatteryState_melodic');
    assert.equal(catalog.dji_M210_melodic.mission, 'v1'); // inherited
  });

  test('the profile overrides that lived in missionSymbols.js are now `mission:` of each file', () => {
    const missionOf = Object.fromEntries(Object.entries(catalog).filter(([, def]) => def.mission).map(([key, def]) => [key, def.mission]));
    assert.deepEqual(missionOf, {
      dji_M210_noetic: 'v1',
      dji_M210_melodic: 'v1',
      px4_mavros: 'v1',
    });
  });
});

describe('extends', () => {
  const base = {
    firmwares: ['px4'],
    vehicleTypes: ['multirotor'],
    subscribers: {
      position: { name: '/gps', messageType: 'sensor_msgs/NavSatFix' },
      battery: { name: '/battery', messageType: 'sensor_msgs/BatteryState', scale: 100 },
    },
  };

  test('a child merges entry by entry, field by field, and replaces everything else', () => {
    const child = { extends: 'base', vehicleTypes: ['vtol'], subscribers: { battery: { messageType: 'x/BatteryState' } } };
    assert.deepEqual(mergeInterface(base, child), {
      extends: 'base',
      firmwares: ['px4'],
      vehicleTypes: ['vtol'],
      subscribers: {
        position: { name: '/gps', messageType: 'sensor_msgs/NavSatFix' },
        battery: { name: '/battery', messageType: 'x/BatteryState', scale: 100 },
      },
    });
  });

  test('a null entry removes the inherited one', () => {
    const { catalog } = buildCatalog({ base, child: { extends: 'base', subscribers: { battery: null } } });
    assert.deepEqual(Object.keys(catalog.child.subscribers), ['position']);
  });

  test('a null entry in an interface that extends nothing is an error', () => {
    assert.deepEqual(problemsOf({ ...base, subscribers: { ...base.subscribers, battery: null } }), [
      ['x', 'a null entry removes an inherited one, and this interface extends nothing'],
    ]);
  });

  test('the order of the files does not matter', () => {
    const { catalog } = buildCatalog({ c: { extends: 'b' }, b: { extends: 'a' }, a: base });
    assert.deepEqual(Object.keys(catalog).sort(), ['a', 'b', 'c']);
    assert.deepEqual(catalog.c.subscribers, base.subscribers);
  });

  test('an unknown parent, a cycle and an invalid parent leave the interface out, saying why', () => {
    const { catalog, problems } = buildCatalog({
      orphan: { ...base, extends: 'nobody' },
      a: { ...base, extends: 'b' },
      b: { ...base, extends: 'a' },
      broken: { subscribers: { position: { nmae: '/x' } } },
      child: { extends: 'broken' },
      ok: base,
    });
    assert.deepEqual(Object.keys(catalog), ['ok']);
    const messages = Object.fromEntries(problems.map((p) => [String(p.path[0]), p.message]));
    assert.equal(messages.orphan, "extends unknown 'nobody'");
    assert.match(messages.b ?? messages.a, /^cycle: /);
    assert.equal(messages.child, "not loaded because its parent 'broken' is invalid");
  });

  test('an abstract base is not in the catalog, its children are, and abstract is not inherited', () => {
    const { catalog, abstract } = buildCatalog({
      dji_base: { abstract: true, subscribers: base.subscribers },
      m: { extends: 'dji_base', firmwares: ['dji'], vehicleTypes: ['multirotor'] },
    });
    assert.deepEqual(Object.keys(catalog), ['m']);
    assert.deepEqual(Object.keys(abstract), ['dji_base']);
    assert.equal(catalog.m.abstract, undefined);
  });

  test('a concrete interface must say its firmwares and the vehicle types it serves', () => {
    assert.deepEqual(problemsOf({ subscribers: base.subscribers }), [
      ['x.firmwares', 'missing firmwares'],
      ['x.vehicleTypes', 'missing vehicleTypes'],
    ]);
    assert.equal(problemsOf({ ...base, vehicleTypes: ['submarine'] }).length, 1);
    assert.equal(problemsOf({ ...base, vehicleTypes: ['vtol', 'vtol'] }).length, 1);
    assert.equal(problemsOf({ ...base, vehicleTypes: [] }).length, 1);
  });

  test('rootOf follows the chain to the top', () => {
    const raw = { a: {}, b: { extends: 'a' }, c: { extends: 'b' } };
    assert.equal(rootOf(raw, 'c'), 'a');
    assert.equal(rootOf(raw, 'a'), 'a');
  });
});

describe('categoryModel and variants', () => {
  test('the device form lists each root before its variants', () => {
    const detailed = categoryModel.getAllDetailed({ protocol: 'ros' });
    const names = detailed.map((c) => c.name);
    assert.ok(names.indexOf('dji_M210_noetic') < names.indexOf('dji_M210_melodic'));
    assert.equal(detailed.find((c) => c.name === 'dji_M210_melodic').root, 'dji_M210_noetic');
    assert.ok(detailed.every((c) => c.name !== 'px4_mavlink')); // ros carries no MAVLink interface
  });

  test('the editor gets the file as written, not the resolved one', () => {
    const raw = categoryModel.getCategoryRaw('dji_M210_melodic');
    assert.equal(raw.extends, 'dji_M210_noetic');
    assert.deepEqual(Object.keys(raw.subscribers), ['battery']);
  });

  test('deleting a parent other interfaces extend is refused before touching the file', async () => {
    await assert.rejects(categoryModel.deleteCategory('dji_M210_noetic'), (err) => {
      assert.equal(err.status, 400);
      assert.ok(err.errors.some((e) => e.path[0] === 'dji_M210_melodic'));
      return true;
    });
    assert.ok(categoryModel.getCategory('dji_M210_melodic'));
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
      assert.match(problems[0].message, /not named after a valid key/);
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
      await assert.rejects(writeInterfaceFile(dir, '../escape', {}), /not a valid key/);
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
    assert.deepEqual(problemsOf({ ...IDENTITY_FIELDS, services: { command_mission: { name: '/start' } } }), [
      ['x.services.command_mission', 'missing messageType'],
    ]);
    assert.deepEqual(problemsOf({ ...IDENTITY_FIELDS, services: { command_mission: { messageType: 'std_srvs/SetBool' } } }), [
      ['x.services.command_mission', 'missing name (the ROS topic/service/action)'],
    ]);
  });

  test('a MAVLink interface has no names and does not mix in ROS messages', () => {
    assert.deepEqual(problemsOf({ ...IDENTITY_FIELDS, subscribers: { battery: { messageType: 'mavlink/BATTERY_STATUS' } } }), []);
    assert.deepEqual(problemsOf({ ...IDENTITY_FIELDS, subscribers: { battery: { name: '/b', messageType: 'mavlink/BATTERY_STATUS' } } }), [
      ['x.subscribers.battery', 'a MAVLink entry has no name (there are no topics)'],
    ]);
    assert.deepEqual(
      problemsOf({
        ...IDENTITY_FIELDS,
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
