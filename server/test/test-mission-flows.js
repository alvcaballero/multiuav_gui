import { test, describe, before, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import sequelize from '../common/sequelize.js';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { missionModel } from '../models/mission/mission.js';
import { missionSMModel } from '../models/mission/missionSM.js';
import { taskScheduler } from '../models/mission/taskScheduler.js';
import { commandsController } from '../controllers/commands.js';
import { ExtAppController } from '../controllers/ExtApp.js';
import { TaskGraphError } from '../models/mission/taskGraph.js';
import { MISSION_STATUS as M, TASK_STATUS as T } from '../config/status.js';

// --- nothing leaves the process ---
for (const k of ['missionReqStart', 'missionReqResult', 'missionReqMedia']) ExtAppController[k] = async () => {};
let cmds = [];
const failFor = new Map(); // deviceId -> command type that fails
commandsController.sendCommandDevice = async ({ deviceId, type, attributes }) => {
  cmds.push({ deviceId, type, attributes });
  return failFor.get(deviceId) === type ? { state: 'error', msg: 'stub failure' } : { state: 'success', msg: 'stub' };
};

// Fake actor registry (see test-task-scheduler.js).
const actors = new Map(); // taskId -> { deviceId, alreadyRunning }
missionSMModel.createActorMission = (deviceId, _m, taskId, opts = {}) => actors.set(taskId, { deviceId, ...opts });
missionSMModel.hasActor = (taskId) => actors.has(taskId);
missionSMModel.deviceHasActorInFlight = (deviceId) => [...actors.values()].some((a) => a.deviceId === deviceId);
const finish = async (taskId) => {
  await missionModel.editTask({ id: taskId, status: T.COMPLETED });
  const { deviceId } = actors.get(taskId);
  actors.delete(taskId);
  eventBus.emitSafe(EVENTS.TASK_DEVICE_RELEASED, { deviceId, taskId });
};

const until = async (fn, ms = 3000) => {
  const end = Date.now() + ms;
  while (Date.now() < end) {
    if (await fn()) return true;
    await new Promise((r) => setTimeout(r, 10));
  }
  return false;
};
const byKey = async (missionId) =>
  Object.fromEntries((await missionModel.getTasks({ missionId })).map((t) => [t.taskKey, t]));
const wp = (x) => [{ type: 'takeoff', pos: [x, 0, 10] }];

// Each test gets devices of its own, so no task of another test can hold them.
let n = 0;
let usv, uav, uav2;
const device = async (prefix) =>
  (await sequelize.models.Device.create({ name: `${prefix}_${++n}`, category: 'px4_ros2', camera: [] })).name;

before(() => taskScheduler.init());
beforeEach(async () => {
  cmds = [];
  failFor.clear();
  [usv, uav, uav2] = [await device('usv'), await device('uav'), await device('uav')];
});

const maritime = () => ({
  name: 'maritime',
  tasks: [
    { task_id: 'T1', device: usv, action: 'NAVIGATE', depends_on: [], params: { max_vel: 3 }, wp: wp(1) },
    { task_id: 'T2', device: uav, action: 'TAKEOFF_AND_SURVEY', depends_on: ['T1'], params: {}, wp: [...wp(2), ...wp(3)] },
    { task_id: 'T3', device: usv, action: 'NAVIGATE', depends_on: ['T2'], params: {}, wp: wp(4) },
  ],
});
const deviceId = async (name) => (await sequelize.models.Device.findOne({ where: { name } })).id;

describe('manual flow', () => {
  test('load creates every task but loads only the roots, each with its one-task mission', async () => {
    const { missionId, results } = await missionModel.loadMissionManual(maritime());
    const t = await byKey(missionId);

    assert.deepEqual(results.map((r) => [r.taskKey, r.state]), [['T1', 'success']]);
    assert.equal(t.T1.status, T.LOADED);
    assert.equal(t.T2.status, T.INIT);
    assert.equal(t.T3.status, T.INIT);
    assert.deepEqual(t.T3.dependsOn, ['T2']);
    assert.equal(t.T2.action, 'TAKEOFF_AND_SURVEY');
    assert.equal(t.T2.totalWp, 2);

    const loads = cmds.filter((c) => c.type === 'loadMission');
    assert.equal(loads.length, 1);
    assert.deepEqual(loads[0].attributes.tasks.map((x) => [x.task_id, x.depends_on]), [['T1', []]]);
    assert.equal((await missionModel.getMissionValue(missionId)).status, M.INIT);
  });

  test('command starts the roots with an attached state machine, then the scheduler runs the rest', async () => {
    const { missionId } = await missionModel.loadMissionManual(maritime());
    const { results } = await missionModel.commandMissionManual(missionId);
    const t = await byKey(missionId);

    assert.deepEqual(results.map((r) => [r.taskKey, r.state]), [['T1', 'success']]);
    assert.equal(t.T1.status, T.COMMANDED);
    assert.deepEqual(actors.get(t.T1.id), { deviceId: await deviceId(usv), alreadyRunning: true });
    assert.equal((await missionModel.getMissionValue(missionId)).status, M.RUNNING);
    assert.ok(!actors.has(t.T2.id), 'T2 waits for T1');

    await finish(t.T1.id);
    assert.ok(await until(() => actors.has(t.T2.id)), 'scheduler dispatches T2 after T1');
    assert.equal(actors.get(t.T2.id).alreadyRunning, undefined, 'dependents run the full load+command machine');
    await finish(t.T2.id);
    assert.ok(await until(() => actors.has(t.T3.id)));
    await finish(t.T3.id);
    assert.ok(await until(async () => (await missionModel.getMissionValue(missionId)).status === M.COMPLETED));
  });

  test('a repeated command does not touch a running mission', async () => {
    const { missionId } = await missionModel.loadMissionManual(maritime());
    await missionModel.commandMissionManual(missionId);
    const again = await missionModel.commandMissionManual(missionId);
    assert.equal(again.state, 'warning');
    assert.equal((await missionModel.getMissionValue(missionId)).status, M.RUNNING);
    assert.equal(cmds.filter((c) => c.type === 'commandMission').length, 1);
  });

  test('a root that fails to load goes to ERROR and skips its whole downstream', async () => {
    failFor.set(await deviceId(usv), 'loadMission');
    const { missionId, results } = await missionModel.loadMissionManual(maritime());
    assert.equal(results[0].state, 'error');
    const t = await byKey(missionId);
    assert.equal(t.T1.status, T.ERROR);
    assert.ok(await until(async () => (await byKey(missionId)).T3.status === T.SKIPPED));
    assert.equal((await byKey(missionId)).T2.status, T.SKIPPED);
    assert.equal((await missionModel.getMissionValue(missionId)).status, M.ERROR);
  });

  test('a root that fails to command goes to ERROR; an independent root still runs', async () => {
    const mission = maritime();
    mission.tasks.push({ task_id: 'T4', device: uav2, depends_on: [], params: {}, wp: wp(9) });
    failFor.set(await deviceId(usv), 'commandMission');
    const { missionId } = await missionModel.loadMissionManual(mission);
    await missionModel.commandMissionManual(missionId);
    assert.ok(await until(async () => (await byKey(missionId)).T3.status === T.SKIPPED));
    const t = await byKey(missionId);
    assert.equal(t.T1.status, T.ERROR);
    assert.equal(t.T4.status, T.COMMANDED);
    assert.equal((await missionModel.getMissionValue(missionId)).status, M.RUNNING);
  });

  test('legacy route[] still loads, one independent task per route', async () => {
    const { missionId, results } = await missionModel.loadMissionManual({
      route: [
        { uav: usv, id: 0, attributes: { max_vel: 7 }, wp: wp(1) },
        { uav: uav, id: 0, attributes: {}, wp: wp(2) },
      ],
    });
    assert.deepEqual(results.map((r) => r.taskKey), ['T1', 'T2']);
    const t = await byKey(missionId);
    assert.equal(t.T1.status, T.LOADED);
    assert.equal(t.T2.status, T.LOADED);
    const plan = await missionModel.getMissionPlan((await missionModel.getMissionValue(missionId)).planId);
    assert.equal(plan.missionData.version, '3', 'stored as received, tagged v3 for the client');
    assert.equal(cmds.find((c) => c.type === 'loadMission').attributes.tasks[0].params.max_vel, 7);
  });

  test('an invalid graph is rejected with a 400 before anything is persisted', async () => {
    const plansBefore = (await missionModel.getAllMissionPlans()).length;
    const bad = maritime();
    bad.tasks[0].depends_on = ['T3'];
    await assert.rejects(missionModel.loadMissionManual(bad), (err) => err instanceof TaskGraphError && err.status === 400);
    assert.equal((await missionModel.getAllMissionPlans()).length, plansBefore);
  });
});

describe('automatic flow (initMission)', () => {
  async function requested() {
    return (await missionModel.createMission({ name: 'auto', externalId: 1000 + ++n })).id;
  }

  test('runs the mission and dispatches only the roots through the scheduler', async () => {
    const missionId = await requested();
    const ok = await missionModel.initMission(missionId, maritime());
    assert.ok(ok);
    const m = await missionModel.getMissionValue(missionId);
    assert.equal(m.status, M.RUNNING);
    assert.ok(m.planId);
    const t = await byKey(missionId);
    assert.deepEqual(Object.keys(t).sort(), ['T1', 'T2', 'T3']);
    assert.ok(actors.has(t.T1.id));
    assert.ok(!actors.has(t.T2.id) && !actors.has(t.T3.id));
    await finish(t.T1.id);
    assert.ok(await until(() => actors.has(t.T2.id)));
  });

  test('legacy planner route[] runs every route at once (no dependencies)', async () => {
    const missionId = await requested();
    await missionModel.initMission(missionId, {
      route: [
        { uav: usv, id: 0, attributes: {}, wp: wp(1) },
        { uav: uav, id: 1, attributes: {}, wp: wp(2) },
      ],
    });
    const t = await byKey(missionId);
    assert.ok(actors.has(t.T1.id) && actors.has(t.T2.id));
  });

  test('a task on an unknown device fails and skips its dependents; the rest runs', async () => {
    const missionId = await requested();
    const mission = maritime();
    mission.tasks[1].device = 'ghost_uav';
    mission.tasks.push({ task_id: 'T4', device: uav2, depends_on: [], params: {}, wp: wp(9) });
    await missionModel.initMission(missionId, mission);
    assert.ok(await until(async () => (await byKey(missionId)).T3.status === T.SKIPPED));
    const t = await byKey(missionId);
    assert.equal(t.T2.status, T.ERROR);
    assert.match(t.T2.errorMessage, /ghost_uav/);
    assert.equal(t.T2.deviceId, null);
    assert.ok(actors.has(t.T1.id) && actors.has(t.T4.id));
  });

  test('an invalid planner graph fails the mission without persisting a plan', async () => {
    const missionId = await requested();
    const bad = maritime();
    bad.tasks[2].device = usv;
    bad.tasks[2].depends_on = [];
    assert.equal(await missionModel.initMission(missionId, bad), false);
    const m = await missionModel.getMissionValue(missionId);
    assert.equal(m.status, M.ERROR);
    assert.match(m.errorMessage, /not ordered/);
    assert.equal(m.planId, null);
  });
});
