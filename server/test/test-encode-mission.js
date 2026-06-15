import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { encodeRosSrv } from '../models/ros/rosEncode.js';

// ─── Fixtures ─────────────────────────────────────────────────────────────────

// Route mínima compartida: 2 waypoints, acciones photo + gimbal
const BASE_WP = [
  {
    pos: [38.13898041, -3.17466429, 9],
    action: { photo: 0, gimbal: -90 },
    yaw: 153.6,
  },
  {
    pos: [38.13816516, -3.17414854, 29.19],
    speed: 5,
  },
];

const BASE_ATTRS = {
  mode_landing: 2,
  mode_yaw: 2,
  mode_gimbal: 0,
  idle_vel: 7,
  max_vel: 12,
  mode_trace: 2,
};

function makeRoute(uav_type, attrs = BASE_ATTRS, wp = BASE_WP) {
  return { uav: 'uav1', uav_type, attributes: attrs, wp };
}

// ─── ROS1: aerialcore_common/ConfigMission ────────────────────────────────────

describe('encodeRosSrv — ROS1 (aerialcore_common/ConfigMission)', () => {
  const route = makeRoute('dji_M210_noetic');
  const msgType = 'aerialcore_common/ConfigMission';

  test('devuelve type=waypoint', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.equal(result.type, 'waypoint');
  });

  test('waypoints tienen la forma correcta (lat/lon/alt)', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.equal(result.waypoint.length, 2);
    assert.ok('latitude' in result.waypoint[0]);
    assert.ok('longitude' in result.waypoint[0]);
    assert.ok('altitude' in result.waypoint[0]);
    assert.equal(result.waypoint[0].latitude, 38.13898041);
    assert.equal(result.waypoint[0].longitude, -3.17466429);
    assert.equal(result.waypoint[0].altitude, 9);
  });

  test('velocidades: idleVel y maxVel de attributes', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.equal(result.idleVel, 7);
    assert.equal(result.maxVel, 12);
  });

  test('speed usa idle_vel en wp sin speed, y speed explícita en wp con speed', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.equal(result.speed.data[0], 7);   // wp[0] sin speed → idle_vel
    assert.equal(result.speed.data[1], 5);   // wp[1] speed: 5
  });

  test('yaw se recoge del wp', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.equal(result.yaw.data[0], 153.6);
    assert.equal(result.yaw.data[1], 0); // wp[1] sin yaw → 0
  });

  test('commandList y commandParameter son arrays aplanados de longitud 20 (2 wp × 10)', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.equal(result.commandList.data.length, 20);
    assert.equal(result.commandParameter.data.length, 20);
  });

  test('modos de vuelo de attributes', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.equal(result.yawMode, 2);
    assert.equal(result.finishAction, 2);
    assert.equal(result.traceMode, 2);
  });

  test('atributos con defaults cuando attributes está vacío', () => {
    const emptyRoute = makeRoute('dji_M210_noetic', {});
    const result = encodeRosSrv({ type: 'configureMission', msg: emptyRoute, msgType });
    assert.equal(result.idleVel, 1.8);
    assert.equal(result.maxVel, 10);
    assert.equal(result.yawMode, 0);
    assert.equal(result.finishAction, 0);
  });

  test('msg null devuelve {}', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: null, msgType });
    assert.deepEqual(result, {});
  });
});

// ─── ROS2: muav_gcs_interfaces/srv/LoadMission ───────────────────────────────

describe('encodeRosSrv — ROS2 (muav_gcs_interfaces/srv/LoadMission)', () => {
  const route = makeRoute('px4_ros2');
  const msgType = 'muav_gcs_interfaces/srv/LoadMission';

  test('tiene wrapper request', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.ok('request' in result);
  });

  test('request tiene campos ROS2 en snake_case', () => {
    const { request } = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.ok('vel_max' in request);
    assert.ok('vel_idle' in request);
    assert.ok('yaw_mode' in request);
    assert.ok('trace_mode' in request);
    assert.ok('finish_action' in request);
    assert.ok('command_list' in request);
    assert.ok('command_parameter' in request);
  });

  test('waypoints correctos dentro de request', () => {
    const { request } = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.equal(request.waypoint.length, 2);
    assert.equal(request.waypoint[0].latitude, 38.13898041);
  });
});

// ─── PSDK: psdk_interfaces/srv/InitWaypointV2Setting ─────────────────────────

describe('encodeRosSrv — PSDK (psdk_interfaces/srv/InitWaypointV2Setting)', () => {
  const route = makeRoute('dji_M300_PSDK');
  const msgType = 'psdk_interfaces/srv/InitWaypointV2Setting';

  test('tiene waypoint_v2_init_settings en la raíz', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.ok('waypoint_v2_init_settings' in result);
  });

  test('mission tiene el número correcto de waypoints', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    const { waypoint_v2_init_settings } = result;
    assert.equal(waypoint_v2_init_settings.mission.length, 2);
    assert.equal(waypoint_v2_init_settings.miss_total_len, 2);
  });

  test('waypoint PSDK tiene campos DJI correctos', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    const wp0 = result.waypoint_v2_init_settings.mission[0];
    assert.ok('longitude' in wp0);
    assert.ok('latitude' in wp0);
    assert.ok('relative_height' in wp0);
    assert.ok('waypoint_type' in wp0);
    assert.ok('heading_mode' in wp0);
    assert.ok('auto_flight_speed' in wp0);
    assert.equal(wp0.latitude, 38.13898041);
    assert.equal(wp0.relative_height, 9);
  });

  test('heading viene del yaw del wp', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    assert.equal(result.waypoint_v2_init_settings.mission[0].heading, 153.6);
    assert.equal(result.waypoint_v2_init_settings.mission[1].heading, 0); // wp[1] sin yaw
  });

  test('speed per-waypoint: usa idle cuando no hay speed, explícita cuando la hay', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    const mission = result.waypoint_v2_init_settings.mission;
    assert.equal(mission[0].auto_flight_speed, 7);  // wp[0] sin speed → idle_vel
    assert.equal(mission[1].auto_flight_speed, 5);  // wp[1] speed: 5
    assert.equal(mission[1].config.use_local_cruise_vel, 1);
    assert.equal(mission[0].config.use_local_cruise_vel, 0);
  });

  test('modos de vuelo de attributes: finished_action, max_flight_speed', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    const init = result.waypoint_v2_init_settings;
    assert.equal(init.finished_action, 2);
    assert.equal(init.max_flight_speed, 12);
    assert.equal(init.auto_flight_speed, 7);
  });

  test('actions se generan para wp con acciones PSDK válidas (photo, gimbal)', () => {
    const result = encodeRosSrv({ type: 'configureMission', msg: route, msgType });
    const actions = result.waypoint_v2_init_settings.actions;
    assert.ok(actions.length > 0);
    assert.equal(result.action_num, actions.length);
    // cada action tiene action_id incremental
    actions.forEach((a, i) => assert.equal(a.action_id, i));
  });

  test('wp sin acciones no genera actions', () => {
    const routeNoActions = makeRoute('dji_M300_PSDK', BASE_ATTRS, [
      { pos: [38.0, -3.0, 10] },
      { pos: [38.1, -3.1, 20] },
    ]);
    const result = encodeRosSrv({ type: 'configureMission', msg: routeNoActions, msgType });
    assert.equal(result.waypoint_v2_init_settings.actions.length, 0);
    assert.equal(result.action_num, 0);
  });

  test('lanza RangeError si mode_yaw tiene valor inválido', () => {
    const badRoute = makeRoute('dji_M300_PSDK', { ...BASE_ATTRS, mode_yaw: 99 });
    assert.throws(
      () => encodeRosSrv({ type: 'configureMission', msg: badRoute, msgType }),
      { name: 'RangeError', message: /yawMode=99/ }
    );
  });

  test('lanza RangeError si mode_landing tiene valor inválido', () => {
    const badRoute = makeRoute('dji_M300_PSDK', { ...BASE_ATTRS, mode_landing: 99 });
    assert.throws(
      () => encodeRosSrv({ type: 'configureMission', msg: badRoute, msgType }),
      { name: 'RangeError', message: /finishAction=99/ }
    );
  });
});

// ─── Servicios no-misión (passthrough / vacío) ────────────────────────────────

describe('encodeRosSrv — servicios no-misión', () => {
  test('std_srvs/TriggerRequest devuelve {}', () => {
    const result = encodeRosSrv({ type: 'commandMission', msg: {}, msgType: 'std_srvs/TriggerRequest' });
    assert.deepEqual(result, {});
  });

  test('psdk_interfaces/srv/StartWaypointV2Mission devuelve {}', () => {
    const result = encodeRosSrv({ type: 'commandMission', msg: { data: true }, msgType: 'psdk_interfaces/srv/StartWaypointV2Mission' });
    assert.deepEqual(result, {});
  });

  test('geometry_msgs/Twist hace passthrough con defaults', () => {
    const result = encodeRosSrv({
      type: 'custom',
      msg: { linear: { x: 1, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0.5 } },
      msgType: 'geometry_msgs/Twist',
    });
    assert.equal(result.linear.x, 1);
    assert.equal(result.angular.z, 0.5);
  });

  test('tipo desconocido devuelve el msg tal cual', () => {
    const msg = { foo: 'bar' };
    const result = encodeRosSrv({ type: 'custom', msg, msgType: 'unknown/Type' });
    assert.deepEqual(result, msg);
  });
});
