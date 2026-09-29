import { test, describe, before } from 'node:test';
import assert from 'node:assert/strict';
import sequelize from '../common/sequelize.js';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { missionModel } from '../models/mission/mission.js';
import { missionSMModel } from '../models/mission/missionSM.js';
import { taskScheduler } from '../models/mission/taskScheduler.js';
import { MISSION_STATUS as M, TASK_STATUS as T } from '../config/status.js';

// Fake actor registry: dispatching a task = "a state machine now runs on its device".
const actors = new Map(); // taskId -> deviceId
missionSMModel.createActorMission = (deviceId, _missionId, taskId) => actors.set(taskId, deviceId);
missionSMModel.hasActor = (taskId) => actors.has(taskId);
missionSMModel.deviceHasActorInFlight = (deviceId) => [...actors.values()].includes(deviceId);
const release = (taskId) => {
  const deviceId = actors.get(taskId);
  actors.delete(taskId);
  eventBus.emitSafe(EVENTS.TASK_DEVICE_RELEASED, { deviceId, taskId });
};

// Close a dispatched task for good, so later tests' device releases don't re-dispatch it.
const finish = async (task) => {
  await missionModel.editTask({ id: task.id, status: T.COMPLETED });
  release(task.id);
};

const until = async (fn, ms = 3000) => {
  const end = Date.now() + ms;
  while (Date.now() < end) {
    if (await fn()) return true;
    await new Promise((r) => setTimeout(r, 10));
  }
  return false;
};
const status = async (task) => (await missionModel.getTask(task.id)).status;

let d = [];
before(async () => {
  for (const name of ['usv_1', 'uav_1', 'uav_2']) {
    d.push((await sequelize.models.Device.create({ name, category: 'px4_ros2', camera: [] })).id);
  }
  taskScheduler.init();
});

async function graph(missionStatus, spec) {
  const mission = await missionModel.createMission({ name: 'sched', trigger: 'manual', status: missionStatus });
  const t = {};
  for (const [taskKey, deviceId, dependsOn = []] of spec) {
    t[taskKey] = await missionModel.createTask({ missionId: mission.id, deviceId, taskKey, dependsOn, status: T.INIT });
  }
  return { mission, t };
}

describe('taskScheduler', () => {
  test('maritime chain USV -> UAV -> USV runs in dependency order', async () => {
    const [usv, uav] = d;
    const { mission, t } = await graph(M.RUNNING, [['T1', usv], ['T2', uav, ['T1']], ['T3', usv, ['T2']]]);

    assert.deepEqual(await taskScheduler.dispatchReady(mission.id), ['T1']);

    await missionModel.editTask({ id: t.T1.id, status: T.COMPLETED });
    assert.ok(await until(() => actors.has(t.T2.id)), 'T2 dispatched once T1 reached its last waypoint');

    await missionModel.editTask({ id: t.T2.id, status: T.COMPLETED });
    assert.deepEqual(await taskScheduler.dispatchReady(mission.id), []);
    assert.ok(!actors.has(t.T3.id), 'T3 waits: T1 state machine still holds the USV');

    release(t.T1.id);
    release(t.T2.id);
    assert.ok(await until(() => actors.has(t.T3.id)), 'T3 dispatched when the USV is released');
    await finish(t.T3);
  });

  test('consecutive tasks on the same device wait for the device, not just the dependency', async () => {
    const [usv] = d;
    const { mission, t } = await graph(M.RUNNING, [['T1', usv], ['T2', usv, ['T1']]]);
    await taskScheduler.dispatchReady(mission.id);

    await missionModel.editTask({ id: t.T1.id, status: T.COMPLETED });
    assert.deepEqual(await taskScheduler.dispatchReady(mission.id), []);

    release(t.T1.id);
    assert.ok(await until(() => actors.has(t.T2.id)));
    await finish(t.T2);
  });

  test('nothing is dispatched until the mission is RUNNING', async () => {
    const [usv] = d;
    const { mission, t } = await graph(M.INIT, [['T1', usv]]);
    assert.deepEqual(await taskScheduler.dispatchReady(mission.id), []);
    await missionModel.editMission({ id: mission.id, status: M.RUNNING });
    assert.deepEqual(await taskScheduler.dispatchReady(mission.id), ['T1']);
    await missionModel.editTask({ id: t.T1.id, status: T.COMPLETED });
    release(t.T1.id);
  });

  test('a failed task skips its whole downstream; an independent branch still runs', async () => {
    const [usv, uav, uav2] = d;
    const { mission, t } = await graph(M.RUNNING, [
      ['T1', usv],
      ['T2', uav, ['T1']],
      ['T3', usv, ['T2']],
      ['T4', uav2],
    ]);
    assert.deepEqual((await taskScheduler.dispatchReady(mission.id)).sort(), ['T1', 'T4']);

    await missionModel.editTask({ id: t.T1.id, status: T.ERROR });
    release(t.T1.id);
    assert.ok(await until(async () => (await status(t.T3)) === T.SKIPPED), 'skip is transitive');
    assert.equal(await status(t.T2), T.SKIPPED);
    assert.match((await missionModel.getTask(t.T2.id)).errorMessage, /T1/);
    assert.equal(await status(t.T4), T.INIT, 'independent branch untouched');

    await missionModel.editTask({ id: t.T4.id, status: T.COMPLETED });
    release(t.T4.id);
    assert.ok(await until(async () => (await missionModel.getMissionValue(mission.id)).status === M.COMPLETED_WITH_ERRORS));
  });

  test('a join waits for every parent', async () => {
    const [usv, uav, uav2] = d;
    const { mission, t } = await graph(M.RUNNING, [['A', usv], ['B', uav, ['A']], ['C', uav2, ['A']], ['D', usv, ['B', 'C']]]);
    await taskScheduler.dispatchReady(mission.id);
    await missionModel.editTask({ id: t.A.id, status: T.COMPLETED });
    release(t.A.id);
    assert.ok(await until(() => actors.has(t.B.id) && actors.has(t.C.id)));

    await missionModel.editTask({ id: t.B.id, status: T.COMPLETED });
    assert.deepEqual(await taskScheduler.dispatchReady(mission.id), []);
    await missionModel.editTask({ id: t.C.id, status: T.COMPLETED });
    assert.ok(await until(() => actors.has(t.D.id)));
    release(t.B.id);
    release(t.C.id);
    await finish(t.D);
  });

  test('concurrent dispatch requests never dispatch a task twice', async () => {
    const [usv] = d;
    const { mission, t } = await graph(M.RUNNING, [['T1', usv]]);
    const calls = [];
    const original = missionSMModel.createActorMission;
    missionSMModel.createActorMission = (...args) => {
      calls.push(args[2]);
      return original(...args);
    };
    const results = await Promise.all([1, 2, 3].map(() => taskScheduler.dispatchReady(mission.id)));
    missionSMModel.createActorMission = original;
    assert.deepEqual(results.flat(), ['T1']);
    assert.deepEqual(calls, [t.T1.id]);
    await finish(t.T1);
  });

  test('a device busy in another mission (task without state machine) delays dispatch until it ends', async () => {
    const [usv] = d;
    const other = await graph(M.RUNNING, [['X', usv]]);
    await missionModel.editTask({ id: other.t.X.id, status: T.RUNNING }); // manual root: no actor

    const { mission, t } = await graph(M.RUNNING, [['T1', usv]]);
    assert.deepEqual(await taskScheduler.dispatchReady(mission.id), []);

    await missionModel.editTask({ id: other.t.X.id, status: T.COMPLETED });
    assert.ok(await until(() => actors.has(t.T1.id)), 'freed device is picked up across missions');
    await finish(t.T1);
  });
});

describe('shouldDownloadFiles', () => {
  test('only the device last open task downloads', async () => {
    const [usv, uav] = d;
    const { t } = await graph(M.INIT, [['T1', usv], ['T2', uav, ['T1']], ['T3', usv, ['T2']]]);
    assert.equal(await missionModel.shouldDownloadFiles(t.T1.id), false);
    assert.equal(await missionModel.shouldDownloadFiles(t.T2.id), true);
    await missionModel.editTask({ id: t.T3.id, status: T.SKIPPED });
    assert.equal(await missionModel.shouldDownloadFiles(t.T1.id), true, 'T3 skipped: T1 is now the last');
  });
});
