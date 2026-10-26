import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { encodeMsg } from '../models/msgEncode.js';
import { categoryModel } from '../models/category.js';

// Encoder PSDK (psdk_interfaces/srv/InitWaypointV2Setting): el gimbal solo se mueve con acciones ROTATE_GIMBAL
// (el mensaje WaypointV2 no tiene campo de gimbal). La orden es ABSOLUTA, así que se emite solo cuando el
// pitch efectivo (action.gimbal, si no wp.gimbal) cambia respecto del último emitido en la misión.

const MSG_TYPE = 'psdk_interfaces/srv/InitWaypointV2Setting';
const ACTUATOR_GIMBAL = 2;
const ACTUATOR_AIRCRAFT = 4;
const ACTUATOR_CAMERA = 1;
const TRIGGER_SAMPLE_REACH_POINT = 5;
const TRIGGER_ACTION_ASSOCIATED = 2;

const ATTRS = { mode_landing: 2, mode_yaw: 2, idle_vel: 7, max_vel: 12, mode_trace: 2 };

function encode(wp) {
  const task = { device: 'uav1', uav_type: 'dji_M300_PSDK', params: ATTRS, wp };
  return encodeMsg(MSG_TYPE, task).waypoint_v2_init_settings.actions;
}

const gimbalActions = (actions) => actions.filter((a) => a.waypoint_v2_action_actuator_type === ACTUATOR_GIMBAL);
const pitchOf = (a) => a.waypoint_v2_gimbal_actuator.waypoint_v2_gimbal_actuator_rotation_param;

describe('MissionToPsdkV2 — gimbal', () => {
  test('action.gimbal -90 → un ROTATE_GIMBAL absoluto con y = -900', () => {
    const gimbals = gimbalActions(encode([{ pos: [38.0, -3.0, 10], action: { gimbal: -90 } }]));
    assert.equal(gimbals.length, 1);
    assert.equal(pitchOf(gimbals[0]).y, -900);
    assert.equal(pitchOf(gimbals[0]).ctrl_mode, 0);
  });

  test('wp.gimbal sin clave action se emite igual', () => {
    const wp = [{ pos: [38.0, -3.0, 10], gimbal: -45 }];
    const gimbals = gimbalActions(encode(wp));
    assert.equal(gimbals.length, 1);
    assert.equal(pitchOf(gimbals[0]).y, -450);
    // el gimbal cuelga del waypoint (o de la cadena de stay), no queda huérfano
    const actions = encode(wp);
    assert.equal(actions.at(-1).waypoint_v2_action_actuator_type, ACTUATOR_AIRCRAFT); // start flying
  });

  test('-90 y luego 0: el 0 es un valor real y se emite', () => {
    const gimbals = gimbalActions(
      encode([
        { pos: [38.0, -3.0, 10], action: { gimbal: -90 } },
        { pos: [38.1, -3.1, 10], action: { gimbal: 0 } },
      ])
    );
    assert.equal(gimbals.length, 2);
    assert.equal(pitchOf(gimbals[0]).y, -900);
    assert.equal(pitchOf(gimbals[1]).y, 0);
  });

  test('el mismo pitch en waypoints consecutivos se emite una sola vez', () => {
    const gimbals = gimbalActions(
      encode([
        { pos: [38.0, -3.0, 10], action: { gimbal: -90 } },
        { pos: [38.1, -3.1, 10], gimbal: -90 },
        { pos: [38.2, -3.2, 10], action: { photo: 0 } }, // sin pitch: conserva el valor
        { pos: [38.3, -3.3, 10], action: { gimbal: -90 } },
      ])
    );
    assert.equal(gimbals.length, 1);
  });

  test('el primer pitch definido siempre se emite, incluso 0', () => {
    const gimbals = gimbalActions(encode([{ pos: [38.0, -3.0, 10], gimbal: 0 }]));
    assert.equal(gimbals.length, 1);
    assert.equal(pitchOf(gimbals[0]).y, 0);
  });

  test('action.gimbal gana sobre wp.gimbal', () => {
    const gimbals = gimbalActions(encode([{ pos: [38.0, -3.0, 10], gimbal: -10, action: { gimbal: -60 } }]));
    assert.equal(gimbals.length, 1);
    assert.equal(pitchOf(gimbals[0]).y, -600);
  });

  test('cadena de un wp: stay → yaw → gimbal → photo → start flying', () => {
    const actions = encode([{ pos: [38.0, -3.0, 10], action: { yaw: 90, gimbal: -30, photo: 0 } }]);
    assert.equal(actions.length, 5);
    const [stay, yaw, gimbal, photo, start] = actions;

    assert.equal(stay.waypoint_v2_action_actuator_type, ACTUATOR_AIRCRAFT);
    assert.equal(stay.waypoint_v2_action_trigger_type, TRIGGER_SAMPLE_REACH_POINT);

    assert.equal(yaw.waypoint_v2_action_actuator_type, ACTUATOR_AIRCRAFT);
    assert.equal(gimbal.waypoint_v2_action_actuator_type, ACTUATOR_GIMBAL);
    assert.equal(photo.waypoint_v2_action_actuator_type, ACTUATOR_CAMERA);
    assert.equal(start.waypoint_v2_action_actuator_type, ACTUATOR_AIRCRAFT);

    // cada eslabón se dispara al terminar el anterior
    const after = (a) => {
      assert.equal(a.waypoint_v2_action_trigger_type, TRIGGER_ACTION_ASSOCIATED);
      return a.waypoint_v2_associate_trigger.action_id_associated;
    };
    assert.equal(after(yaw), stay.action_id);
    assert.equal(after(gimbal), yaw.action_id);
    assert.equal(after(photo), gimbal.action_id);
    assert.equal(after(start), photo.action_id);
  });
});

// mode_gimbal: WaypointV2 no interpola el pitch entre waypoints, solo mueve el gimbal con acciones (= FREE).
describe('MissionToPsdkV2 — mode_gimbal', () => {
  const encodeWith = (params) =>
    encodeMsg(MSG_TYPE, { device: 'uav1', uav_type: 'dji_M300_PSDK', params, wp: [{ pos: [38.0, -3.0, 10] }] });

  test('PSDK ofrece solo FREE y el default pasa a FREE (el del catálogo es WAYPOINT)', () => {
    const options = categoryModel.getAtributesParam({ type: 'dji_M300_PSDK', param: 'mode_gimbal' });
    assert.deepEqual(
      options.map((o) => o.id),
      [0]
    );
    assert.equal(categoryModel.getAttributesDefaults('dji_M300_PSDK').mode_gimbal, 0);
  });

  test('ConfigMission (M300 OSDK) sigue ofreciendo los dos modos', () => {
    const options = categoryModel.getAtributesParam({ type: 'dji_M300', param: 'mode_gimbal' });
    assert.deepEqual(
      options.map((o) => o.id),
      [0, 1]
    );
  });

  test('mode_gimbal 0 (lo que manda Smart-Flies) y sin mode_gimbal se encodean', () => {
    assert.doesNotThrow(() => encodeWith({ ...ATTRS, mode_gimbal: 0 }));
    assert.doesNotThrow(() => encodeWith(ATTRS));
  });

  test('mode_gimbal 1 (WAYPOINT) no se vuela como FREE: el encoder lo rechaza', () => {
    assert.throws(() => encodeWith({ ...ATTRS, mode_gimbal: 1 }), { name: 'RangeError' });
  });
});
