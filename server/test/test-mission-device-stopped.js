import { test, describe, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { missionSMModel } from '../models/mission/missionSM.js';
import { missionController } from '../controllers/mission.js';
import { MISSION_STOP_REASON, TASK_STATUS as T } from '../config/status.js';

// A vehicle that ends its mission on its own before the last waypoint (DJI WaypointV2 stop event) must
// close its task: left in RunningMission it waits for a finish that never comes, and the scheduler keeps
// the device busy until a restart. Real state machine; only the controller's DB writes are stubbed.

let edits = [];
missionController.editTask = async (task) => {
  edits.push(task);
  return task;
};
missionController.updateMission = () => {};

const until = async (fn, ms = 2000) => {
  const end = Date.now() + ms;
  while (Date.now() < end) {
    if (fn()) return true;
    await new Promise((r) => setTimeout(r, 10));
  }
  return false;
};

let nextTask = 70000;
const running = (deviceId) => {
  const taskId = nextTask++;
  missionSMModel.createActorMission(deviceId, 1, taskId, { alreadyRunning: true });
  return taskId;
};

beforeEach(() => {
  edits = [];
});

describe('missionSMModel.deviceStoppedMission', () => {
  test('a preempted mission marks the task ERROR with the reason and frees the device', async () => {
    const taskId = running(501);
    const released = [];
    const onRelease = (r) => released.push(r);
    eventBus.on(EVENTS.TASK_DEVICE_RELEASED, onRelease);
    try {
      const stop = { deviceId: 501, reason: MISSION_STOP_REASON.PREEMPTED, message: 'Mission stopped: preempted' };
      assert.equal(missionSMModel.deviceStoppedMission(stop), true);
      assert.ok(await until(() => released.length === 1), 'the device was not released');
    } finally {
      eventBus.off(EVENTS.TASK_DEVICE_RELEASED, onRelease);
    }
    assert.equal(edits.length, 1);
    assert.equal(edits[0].id, taskId);
    assert.equal(edits[0].status, T.ERROR);
    assert.equal(edits[0].errorMessage, 'Mission stopped: preempted');
    assert.equal(missionSMModel.deviceHasActorInFlight(501), false);
  });

  test('a stop asked from outside is CANCELLED, not an error', async () => {
    running(502);
    const stop = { deviceId: 502, reason: MISSION_STOP_REASON.USER, message: 'Mission stopped' };
    missionSMModel.deviceStoppedMission(stop);
    assert.ok(await until(() => edits.length === 1));
    assert.equal(edits[0].status, T.CANCELLED);
  });

  test('a device with no running task is only a warning', () => {
    const stop = { deviceId: 503, reason: MISSION_STOP_REASON.PREEMPTED, message: 'x' };
    assert.equal(missionSMModel.deviceStoppedMission(stop), false);
    assert.equal(edits.length, 0);
  });

  test('init wires the eventBus signal positionsModel emits', async () => {
    running(504);
    missionSMModel.init();
    const stop = { deviceId: 504, reason: MISSION_STOP_REASON.TAKEOFF_TIMEOUT, message: 'm' };
    eventBus.emitSafe(EVENTS.DEVICE_MISSION_STOPPED, stop);
    assert.ok(await until(() => edits.length === 1));
    assert.equal(edits[0].status, T.ERROR);
  });
});
