import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { missionProblems, formatProblem, PROBLEM_LEVEL, MissionValidationError, assertMissionValid } from '../models/mission/missionValidator.js';
import { commandsModel } from '../models/commands/commands.js';
import { CommandType } from '../config/commandTypes.js';

// A mission may only carry what its interface encodes AND its vehicle can do
// (VEHICLE_PLATFORMS_PLAN.md › F5 · Contenido de la misión). Devices are plain objects: no DB.
const DEVICES = {
  x500_1: { id: 901, name: 'x500_1', category: 'px4_mavlink', protocol: 'mavlink', deviceModel: 'x500', attributes: { systemId: 1 } },
  vtol_1: { id: 902, name: 'vtol_1', category: 'px4_mavlink', protocol: 'mavlink', deviceModel: 'standard_vtol', attributes: { systemId: 2 } },
  vtol_cam: {
    id: 903,
    name: 'vtol_cam',
    category: 'px4_mavlink',
    protocol: 'mavlink',
    deviceModel: 'standard_vtol',
    attributes: { systemId: 3, capabilities: { camera: true } },
  },
  griffin_1: { id: 904, name: 'griffin_1', category: 'ardupilot_mavlink', protocol: 'mavlink', deviceModel: 'griffin', attributes: { systemId: 4 } },
};
const findDevice = async (name) => DEVICES[name] ?? null;

const wp = (action, extra = {}) => ({ pos: [37.4, -6.0, 30], ...(action ? { action } : {}), ...extra });
const mission = (device, { params = {}, wps = [wp(), wp()] } = {}) => ({
  tasks: [{ task_id: 'T1', device, depends_on: [], params, wp: wps }],
});
const errorsOf = async (m) => (await missionProblems(m, findDevice)).problems.filter((p) => p.level === PROBLEM_LEVEL.ERROR);

describe('missionValidator: what a device cannot carry is an error, located', () => {
  test('a mission within the device is valid', async () => {
    assert.deepEqual(await missionProblems(mission('x500_1', { params: { mode_landing: 2 }, wps: [wp({ stay: 5 }), wp()] }), findDevice), {
      ok: true,
      problems: [],
    });
  });

  test('equipment: an action of a capability the device lacks, with where and why', async () => {
    const [problem] = await errorsOf(mission('x500_1', { wps: [wp(), wp({ gimbal: -90 })] }));
    assert.deepEqual(
      { task: problem.task, device: problem.device, wp: problem.wp, field: problem.field },
      { task: 'T1', device: 'x500_1', wp: 1, field: 'action.gimbal' }
    );
    assert.match(problem.message, /needs capability 'gimbal', which x500_1 \(model x500\) does not have/);
    assert.equal(formatProblem(problem).startsWith('T1 · wp 2 · action.gimbal: '), true, 'shown 1-based');
  });

  test('equipment: the camera is its own capability, enabled per device', async () => {
    assert.match((await errorsOf(mission('vtol_1', { wps: [wp({ photo: 1 }), wp()] })))[0].message, /needs capability 'camera'/);
    assert.deepEqual(await errorsOf(mission('vtol_cam', { wps: [wp({ photo: 1 }), wp()] })), []);
  });

  test('a waypoint param of a capability (wp.gimbal) is checked too', async () => {
    const [problem] = await errorsOf(mission('vtol_1', { wps: [wp(null, { gimbal: -45 }), wp()] }));
    assert.equal(problem.field, 'gimbal');
    assert.match(problem.message, /needs capability 'gimbal'/);
  });

  test('vehicle type: a VTOL transition only on a VTOL', async () => {
    const [problem] = await errorsOf(mission('x500_1', { wps: [wp({ vtol_transition: 0 }), wp()] }));
    assert.match(problem.message, /'vtol_transition' is only for vtol; x500_1 is a multirotor/);
    assert.deepEqual(await errorsOf(mission('vtol_1', { wps: [wp({ vtol_transition: 0 }), wp()] })), []);
  });

  test('what the interface does not encode: a param of another profile', async () => {
    const [problem] = await errorsOf(mission('x500_1', { params: { mode_gimbal: 0 } }));
    assert.equal(problem.field, 'params.mode_gimbal');
    assert.match(problem.message, /its interface 'px4_mavlink' does not encode 'mode_gimbal'/);
  });

  test('values: an option the device is not offered, a number out of range', async () => {
    assert.match((await errorsOf(mission('x500_1', { params: { mode_trace: 0 } })))[0].message, /0 is not an option here/);
    assert.match((await errorsOf(mission('x500_1', { wps: [wp({ stay: 9999 }), wp()] })))[0].message, /above the maximum 600/);
    assert.match((await errorsOf(mission('vtol_1', { wps: [wp({ vtol_transition: 7 }), wp()] })))[0].message, /7 is not an option/);
  });

  test('an unknown action is an error, not skipped', async () => {
    assert.match((await errorsOf(mission('x500_1', { wps: [wp({ jump: 1 }), wp()] })))[0].message, /'jump' is not a mission actions/);
  });

  test('a vehicle that does not fly missions', async () => {
    const [problem] = await errorsOf(mission('griffin_1'));
    assert.equal(problem.field, 'device');
    assert.match(problem.message, /griffin_1 does not fly missions/);
  });

  test('an unknown device is a WARNING: the load goes on and that task fails, as before', async () => {
    const { ok, problems } = await missionProblems(mission('ghost'), findDevice);
    assert.equal(ok, true);
    assert.deepEqual(
      problems.map((p) => [p.level, p.field]),
      [[PROBLEM_LEVEL.WARNING, 'device']]
    );
  });

  test('an invalid task graph is reported as problems, not thrown', async () => {
    const { ok, problems } = await missionProblems({ tasks: [] }, findDevice);
    assert.equal(ok, false);
    assert.equal(problems[0].field, 'tasks');
  });

  test('assertMissionValid throws a 400 with every error', async () => {
    const bad = mission('x500_1', { wps: [wp({ gimbal: -90 }), wp({ vtol_transition: 0 })] });
    await assert.rejects(assertMissionValid(bad, findDevice), (err) => {
      assert.ok(err instanceof MissionValidationError);
      assert.equal(err.status, 400);
      assert.equal(err.errors.length, 2);
      return true;
    });
  });
});

describe('missionValidator: the bare LoadMission (MCP load_mission_to_uav) is refused before sending', () => {
  test('sendToDevice answers the problem and never reaches the gateway', async () => {
    const endpoint = { block: 'services', key: 'configure_mission', msgType: 'mavlink/PX4_MISSION' };
    const route = [{ uav: 'x500_1', wp: [wp({ gimbal: -90 }), wp()] }];
    const response = await commandsModel.sendToDevice(DEVICES.x500_1, CommandType.LOAD_MISSION, endpoint, route);
    assert.equal(response.state, 'error');
    assert.match(response.msg, /^Mission refused: T1 · wp 1 · action\.gimbal: .*needs capability 'gimbal'/);
  });
});
