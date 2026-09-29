import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import {
  normalizeMission,
  validateTaskGraph,
  topologicalOrder,
  descendants,
  readyTasks,
  isLastTaskOfDevice,
  missionForTask,
  TaskGraphError,
  TASK_GRAPH_VERSION,
  LEGACY_ROUTE_ACTION,
} from '../models/mission/taskGraph.js';

const wp = (x = 0) => [{ type: 'takeoff', pos: [x, 0, 10] }];
const task = (task_id, device, depends_on = [], extra = {}) => ({ task_id, device, depends_on, wp: wp(), ...extra });

// USV -> UAV -> USV, the reference maritime inspection example.
const maritime = () => [
  task('T1', 'usv_1', [], { action: 'NAVIGATE' }),
  task('T2', 'uav_1', ['T1'], { action: 'TAKEOFF_AND_SURVEY' }),
  task('T3', 'usv_1', ['T2'], { action: 'NAVIGATE' }),
];

describe('normalizeMission — legacy route[] adapter', () => {
  const legacy = {
    name: 'm',
    global_origin: { latitude: 37, longitude: -6, altitude: 0 },
    version: '3',
    route: [
      { name: 'r0', uav: 'px4_1', id: 0, uav_type: 'px4_ros2', attributes: { max_vel: 12 }, wp: wp(1) },
      { name: 'r1', uav: 'px4_2', id: 0, uav_type: 'px4_ros2', attributes: { max_vel: 8 }, wp: wp(2) },
    ],
  };

  test('converts each route into an independent task and bumps the version', () => {
    const m = normalizeMission(legacy);
    assert.equal(m.version, TASK_GRAPH_VERSION);
    assert.equal(m.route, undefined);
    assert.deepEqual(m.global_origin, legacy.global_origin);
    assert.deepEqual(
      m.tasks.map((t) => [t.task_id, t.device, t.action, t.depends_on]),
      [
        ['T1', 'px4_1', LEGACY_ROUTE_ACTION, []],
        ['T2', 'px4_2', LEGACY_ROUTE_ACTION, []],
      ]
    );
  });

  test('moves attributes to params and keeps the rest of the route', () => {
    const [t1] = normalizeMission(legacy).tasks;
    assert.deepEqual(t1.params, { max_vel: 12 });
    assert.equal(t1.attributes, undefined);
    assert.equal(t1.uav, undefined);
    assert.equal(t1.name, 'r0');
    assert.equal(t1.uav_type, 'px4_ros2');
    assert.deepEqual(t1.wp, wp(1));
  });

  test('task_id comes from the index, so duplicate route ids do not collide', () => {
    const ids = normalizeMission(legacy).tasks.map((t) => t.task_id);
    assert.deepEqual(ids, ['T1', 'T2']);
  });

  test('does not mutate the input', () => {
    const input = structuredClone(legacy);
    normalizeMission(input);
    assert.deepEqual(input, legacy);
  });
});

describe('normalizeMission — tasks[]', () => {
  test('accepts the maritime example and defaults missing depends_on', () => {
    const tasks = maritime();
    delete tasks[0].depends_on;
    const m = normalizeMission({ name: 'm', tasks });
    assert.deepEqual(m.tasks[0].depends_on, []);
    assert.equal(m.tasks.length, 3);
  });

  test('rejects both or neither of tasks[]/route[]', () => {
    assert.throws(() => normalizeMission({ tasks: maritime(), route: [] }), /both tasks\[\] and route\[\]/);
    assert.throws(() => normalizeMission({ name: 'm' }), /must have tasks\[\] or route\[\]/);
    assert.throws(() => normalizeMission(null), TaskGraphError);
  });

  test('throws a TaskGraphError carrying every error and a 400 status', () => {
    try {
      normalizeMission({ tasks: [task('T1', ''), task('T1', 'uav_1')] });
      assert.fail('expected throw');
    } catch (err) {
      assert.ok(err instanceof TaskGraphError);
      assert.equal(err.status, 400);
      assert.equal(err.errors.length, 2);
    }
  });
});

describe('validateTaskGraph', () => {
  test('valid graph has no errors', () => {
    assert.deepEqual(validateTaskGraph(maritime()), []);
  });

  test('empty or non-array tasks', () => {
    assert.deepEqual(validateTaskGraph([]), ['tasks must be a non-empty array']);
    assert.deepEqual(validateTaskGraph(undefined), ['tasks must be a non-empty array']);
  });

  test('structural errors', () => {
    const errors = validateTaskGraph([
      { task_id: '', device: 'uav_1', depends_on: [], wp: wp() },
      { task_id: 'T2', device: 'uav_2', depends_on: 'T1', wp: [] },
    ]);
    assert.ok(errors.some((e) => /task_id must be a non-empty string/.test(e)));
    assert.ok(errors.some((e) => /T2: wp must be a non-empty array/.test(e)));
    assert.ok(errors.some((e) => /T2: depends_on must be an array/.test(e)));
  });

  test('duplicate task_id', () => {
    assert.deepEqual(validateTaskGraph([task('T1', 'a'), task('T1', 'b')]), ['T1: duplicate task_id']);
  });

  test('rejects route-style attributes without params', () => {
    const errors = validateTaskGraph([task('T1', 'a', [], { attributes: { max_vel: 5 } })]);
    assert.deepEqual(errors, ["T1: uses 'attributes'; tasks use 'params'"]);
  });

  test('self and unknown dependencies', () => {
    const errors = validateTaskGraph([task('T1', 'a', ['T1']), task('T2', 'b', ['T9'])]);
    assert.deepEqual(errors, ['T1: depends on itself', "T2: depends on unknown task 'T9'"]);
  });

  test('cycle', () => {
    const errors = validateTaskGraph([task('T1', 'a', ['T3']), task('T2', 'b', ['T1']), task('T3', 'c', ['T2'])]);
    assert.deepEqual(errors, ['dependency cycle among tasks: T1, T2, T3']);
  });

  test('two tasks on the same device must be ordered', () => {
    const errors = validateTaskGraph([task('T1', 'usv_1'), task('T2', 'uav_1'), task('T3', 'usv_1', ['T2'])]);
    assert.deepEqual(errors, ["device 'usv_1': tasks T1 and T3 are not ordered by depends_on"]);
  });

  test('transitive ordering through another device is enough', () => {
    assert.deepEqual(validateTaskGraph(maritime()), []);
  });
});

describe('graph helpers', () => {
  const diamond = [task('A', 'x'), task('B', 'y', ['A']), task('C', 'z', ['A']), task('D', 'x', ['B', 'C'])];

  test('topologicalOrder respects dependencies', () => {
    const order = topologicalOrder(diamond);
    assert.equal(order.length, 4);
    const pos = Object.fromEntries(order.map((id, i) => [id, i]));
    assert.ok(pos.A < pos.B && pos.A < pos.C && pos.B < pos.D && pos.C < pos.D);
  });

  test('descendants are transitive and exclude the task itself', () => {
    assert.deepEqual(descendants(diamond, 'A').sort(), ['B', 'C', 'D']);
    assert.deepEqual(descendants(diamond, 'B'), ['D']);
    assert.deepEqual(descendants(diamond, 'D'), []);
  });

  test('readyTasks needs every dependency done', () => {
    const pending = new Set(['B', 'C', 'D']);
    assert.deepEqual(
      readyTasks(diamond, { pending, done: new Set(['A']) }).map((t) => t.task_id),
      ['B', 'C']
    );
    assert.deepEqual(readyTasks(diamond, { pending: new Set(['D']), done: new Set(['A', 'B']) }), []);
    assert.deepEqual(
      readyTasks(diamond, { pending: new Set(['D']), done: new Set(['A', 'B', 'C']) }).map((t) => t.task_id),
      ['D']
    );
  });

  test('readyTasks ignores tasks that are not pending', () => {
    assert.deepEqual(readyTasks(diamond, { pending: new Set(), done: new Set(['A']) }), []);
  });

  test('missionForTask keeps mission fields, one task, no dependencies', () => {
    const mission = normalizeMission({ name: 'm', global_origin: { latitude: 1 }, tasks: maritime() });
    const single = missionForTask(mission, 'T3');
    assert.equal(single.name, 'm');
    assert.deepEqual(single.global_origin, { latitude: 1 });
    assert.deepEqual(single.tasks.map((t) => [t.task_id, t.device, t.depends_on]), [['T3', 'usv_1', []]]);
    assert.deepEqual(validateTaskGraph(single.tasks), []);
    assert.deepEqual(mission.tasks[2].depends_on, ['T2']);
    assert.equal(missionForTask(mission, 'nope'), null);
  });

  test('isLastTaskOfDevice', () => {
    const tasks = maritime();
    assert.equal(isLastTaskOfDevice(tasks, 'T1'), false);
    assert.equal(isLastTaskOfDevice(tasks, 'T2'), true);
    assert.equal(isLastTaskOfDevice(tasks, 'T3'), true);
    assert.equal(isLastTaskOfDevice(tasks, 'nope'), false);
  });
});
