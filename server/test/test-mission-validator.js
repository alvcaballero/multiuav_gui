import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { missionProblems, formatProblem, PROBLEM_LEVEL, MissionValidationError, assertMissionValid } from '../models/mission/missionValidator.js';
import { commandsModel } from '../models/commands/commands.js';
import { CommandType } from '../config/commandTypes.js';

// A mission may only carry what its interface encodes AND its vehicle can do
// (docs/decisions/vehicle-platforms.md › F5 · Contenido de la misión). Devices are plain objects: no DB.
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

describe('missionValidator: geometry on the wing (warnings, from the device model numbers)', () => {
  const at = (lat, lon, alt, action) => ({ pos: [lat, lon, alt], ...(action ? { action } : {}) });
  const warningsOf = async (m) => (await missionProblems(m, findDevice)).problems.filter((p) => p.level === PROBLEM_LEVEL.WARNING);

  // The user's first SITL mission: ~10 m legs at 5 m. PX4 accepted it and never got onto the wing.
  const FIRST_SITL_MISSION = [
    at(47.39797036448863, 8.546164241430802, 5),
    at(47.397990519078064, 8.546264483109354, 5, { vtol_transition: 0 }),
    at(47.39788504330784, 8.546260513142357, 5),
    at(47.39797036448863, 8.546162256447275, 5, { vtol_transition: 1 }),
  ];
  // test/e2e/test-vtol-mission-sitl.js: flown end to end in SITL, wing and back.
  const E2E_MISSION = [
    at(47.3979714, 8.5461635, 40),
    at(47.3979714, 8.54695975511529, 40, { vtol_transition: 0 }),
    at(47.3979714, 8.553595214409375, 40),
    at(47.40156464469997, 8.553595214409375, 40),
    at(47.40156464469997, 8.54695975511529, 40, { vtol_transition: 1 }),
    at(47.39833072447, 8.5461635, 40),
  ];

  test('the mission that flew in SITL raises nothing', async () => {
    assert.deepEqual(await warningsOf(mission('vtol_1', { params: { mode_landing: 2 }, wps: E2E_MISSION })), []);
  });

  test('the mission that never got onto the wing: legs too short for the transition and the turn, too low', async () => {
    const warnings = await warningsOf(mission('vtol_1', { wps: FIRST_SITL_MISSION }));
    const text = warnings.map(formatProblem).join('\n');
    assert.match(text, /T1 · wp 2 · geometry: the leg to wp 3 is 12 m on the wing; it needs ~104 m \(~50 m to reach 10 m\/s on the wing, ~54 m for the 141° turn at wp 3\)/);
    assert.match(text, /T1 · wp 3 · geometry: the leg to wp 4 is 12 m on the wing; it needs ~54 m/);
    assert.match(text, /wp 2 · pos: 5 m is low to fly on the wing \(minimum 20 m over the takeoff\)/);
    // Back on the rotors at the LAST waypoint: no leg after it to brake on (it lands there).
    assert.ok(!warnings.some((w) => /brakes/.test(w.message)));
    assert.ok(warnings.every((w) => w.level === PROBLEM_LEVEL.WARNING), 'estimates: never errors');
    assert.equal((await missionProblems(mission('vtol_1', { wps: FIRST_SITL_MISSION }), findDevice)).ok, true, 'it still loads');
  });

  test('braking back to the rotors needs ~v²/2a after the transition', async () => {
    const wps = [...E2E_MISSION.slice(0, 5), at(47.40156464469997, 8.547, 40)]; // ~3 m after the back transition
    const [brake] = (await warningsOf(mission('vtol_1', { wps }))).filter((w) => w.field === 'action.vtol_transition');
    assert.match(brake.message, /brakes from 15 m\/s in ~56 m, but wp 6 is \d m away/);
  });

  test('a U-turn on the wing needs a loiter', async () => {
    const wps = [E2E_MISSION[0], E2E_MISSION[1], E2E_MISSION[2], at(47.3979714, 8.548, 40), at(47.3979714, 8.5465, 40, { vtol_transition: 1 })];
    const turns = (await warningsOf(mission('vtol_1', { wps }))).filter((w) => /too sharp/.test(w.message));
    assert.equal(turns.length, 1);
    assert.match(turns[0].message, /a 180° turn is too sharp to fly on the wing \(radius 19\.2 m\): it needs a loiter/);
  });

  test('a multirotor gets no wing geometry; heights in AMSL are not compared', async () => {
    assert.deepEqual(await warningsOf(mission('x500_1', { wps: FIRST_SITL_MISSION.map((w) => ({ pos: w.pos })) })), []);
    const amsl = mission('vtol_1', { wps: E2E_MISSION.map((w) => ({ ...w, pos: [w.pos[0], w.pos[1], 5] })) });
    amsl.tasks[0].altitude_ref = 'amsl';
    assert.ok(!(await warningsOf(amsl)).some((w) => w.field === 'pos'));
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
