import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { readDataFile } from '../common/utils.js';
import { missionSchema } from '../config/config.js';
import { categoryModel } from '../models/category.js';
import { toPsdkValue } from '../models/ros/psdkEncode.js';
import { toConfigValue } from '../models/MissionDecoder.js';
import { encodeRosSrv } from '../models/ros/rosEncode.js';
import { profileFor } from '../models/ros/missionSymbols.js';

// ─── Contract tests: the enriched catalog is internally consistent ────────────
// These guard the DOMAIN catalog (symbols, payloads, profiles). The catalog↔firmware
// symbol mapping is asserted in test-encode-mission.js where the encoders live.

const schema = readDataFile(missionSchema);

describe('mission_schema catalog — symbols', () => {
  test('every select option has a unique, non-empty key (symbol)', () => {
    for (const [pname, p] of Object.entries(schema.params)) {
      if (p.type !== 'select') continue;
      const keys = p.options.map((o) => o.key);
      keys.forEach((k) => assert.ok(k && typeof k === 'string', `${pname} option missing key`));
      assert.equal(new Set(keys).size, keys.length, `${pname} has duplicate option keys`);
      // every option also carries the numeric value used on the wire
      p.options.forEach((o) => assert.ok(Number.isInteger(o.value), `${pname} option ${o.key} missing value`));
    }
  });

  test('every action has a unique key and id', () => {
    const entries = Object.values(schema.actions);
    const keys = entries.map((a) => a.key);
    const ids = entries.map((a) => a.id);
    keys.forEach((k) => assert.ok(k?.startsWith('ACTION_'), `action key '${k}' should start with ACTION_`));
    assert.equal(new Set(keys).size, keys.length, 'duplicate action keys');
    assert.equal(new Set(ids).size, ids.length, 'duplicate action ids');
  });
});

describe('mission_schema catalog — payloads', () => {
  test('action payloads (when present) declare type and a numeric default', () => {
    for (const [name, a] of Object.entries(schema.actions)) {
      if (a.payload == null) continue;
      assert.ok(['number', 'bool', 'select'].includes(a.payload.type), `${name} payload bad type`);
      if (a.payload.type === 'number') {
        assert.ok(typeof a.payload.default === 'number', `${name} payload missing numeric default`);
        assert.ok(a.payload.min <= a.payload.max, `${name} payload min>max`);
        assert.ok(a.payload.default >= a.payload.min && a.payload.default <= a.payload.max,
          `${name} payload default out of [min,max]`);
      }
    }
  });

  test('number params declare min<=max and an in-range default', () => {
    for (const [name, p] of Object.entries(schema.params)) {
      if (p.type !== 'number') continue;
      assert.ok(p.min <= p.max, `${name} min>max`);
      assert.ok(p.default >= p.min && p.default <= p.max, `${name} default out of [min,max]`);
    }
  });
});

describe('mission_schema catalog — profiles', () => {
  test('every profile references only existing actions and params', () => {
    for (const [pname, profile] of Object.entries(schema.profiles)) {
      profile.actions.forEach((k) => assert.ok(schema.actions[k], `profile ${pname}: unknown action '${k}'`));
      profile.params.forEach((k) => assert.ok(schema.params[k], `profile ${pname}: unknown param '${k}'`));
    }
  });

  test('select defaults reference a real option value', () => {
    for (const [name, p] of Object.entries(schema.params)) {
      if (p.type !== 'select') continue;
      assert.ok(p.options.some((o) => o.value === p.default), `${name} default ${p.default} not an option value`);
    }
  });
});

describe('mission_schema catalog — category resolution', () => {
  test('symbolForValue resolves the canonical symbol from a wire number', () => {
    assert.equal(categoryModel.symbolForValue('mode_yaw', 0), 'HEADING_MODE_AUTO');
    assert.equal(categoryModel.symbolForValue('mode_landing', 2), 'MISSION_FINISHED_AUTO_LANDING');
    assert.equal(categoryModel.symbolForValue('mode_trace', 3), 'FLIGHT_PATH_MODE_COORDINATE_TURN');
  });

  test('getAttributesDefaults returns the catalog defaults (SSOT)', () => {
    // 'default' profile: idle_vel + the three modes (no max_vel).
    const d = categoryModel.getAttributesDefaults('dji_M210_noetic');
    assert.equal(d.idle_vel, schema.params.idle_vel.default);
    assert.equal(d.mode_yaw, schema.params.mode_yaw.default);
    // max_vel lives only in v2/psdk profiles.
    const v2 = categoryModel.getAttributesDefaults('dji_M300');
    assert.equal(v2.max_vel, schema.params.max_vel.default);
  });

  test('getActions superset keeps the legacy `param` flag and adds key/payload', () => {
    // v2/psdk profiles expose actions; 'default' has none.
    const actions = categoryModel.getActions({ type: 'dji_M300' });
    const gimbal = actions.find((a) => a.name === 'gimbal');
    assert.equal(gimbal.param, true); // back-compat flag
    assert.equal(gimbal.key, 'ACTION_GIMBAL');
    assert.equal(gimbal.payload.unit, 'deg');
    const photo = actions.find((a) => a.name === 'photo');
    assert.equal(photo.param, false); // no payload
    assert.equal(photo.payload, null);
  });

  test('the default profile exposes no actions (by design)', () => {
    // dji_M600 / agv resolve to the bare 'default' profile.
    assert.equal(categoryModel.getActions({ type: 'dji_M600' }).length, 0);
  });
});

// ─── Symbol translation: catalog ↔ PSDK firmware ──────────────────────────────

// Mode params live in either `params` (route-level) or `waypoint_params` (per-wp).
const modeDef = (param) => schema.params[param] ?? schema.waypoint_params[param];

describe('mission_schema ↔ PSDK — symbol translation', () => {
  // The chain is number → symbol (catalog) → firmware number (PSDK). The catalog
  // value and the PSDK number need NOT coincide (e.g. mode_turn): the SYMBOL is the
  // bridge. We assert the catalog value resolves to its declared symbol, and that
  // symbol is mappable by the encoder.
  for (const param of ['mode_yaw', 'mode_trace', 'mode_landing', 'mode_turn']) {
    test(`${param}: value → symbol matches the catalog key, and symbol maps to a number`, () => {
      for (const opt of modeDef(param).options) {
        const symbol = categoryModel.symbolForValue(param, opt.value);
        assert.equal(symbol, opt.key, `${param}=${opt.value} should resolve to ${opt.key}`);
        const firmwareNum = toPsdkValue(param, symbol);
        assert.ok(Number.isInteger(firmwareNum), `${param} ${symbol} should map to a firmware number`);
      }
    });
  }

  test('every catalog mode symbol is mapped by the PSDK encoder', () => {
    for (const param of ['mode_yaw', 'mode_trace', 'mode_landing', 'mode_turn']) {
      for (const opt of modeDef(param).options) {
        assert.doesNotThrow(() => toPsdkValue(param, opt.key), `encoder missing mapping for ${opt.key}`);
      }
    }
  });

  test('mode_turn is intentionally NOT identity (catalog value ≠ PSDK number)', () => {
    // Proves the symbol layer decouples the numbers: catalog CW=1 but PSDK CW=0.
    const cw = modeDef('mode_turn').options.find((o) => o.key === 'TURN_MODE_CLOCKWISE');
    assert.equal(cw.value, 1); // catalog wire value
    assert.equal(toPsdkValue('mode_turn', 'TURN_MODE_CLOCKWISE'), 0); // PSDK firmware number
  });

  test('toPsdkValue throws on an unknown symbol', () => {
    assert.throws(() => toPsdkValue('mode_yaw', 'HEADING_MODE_NONSENSE'), { name: 'RangeError' });
  });
});

describe('per-family option filtering — UI only offers supported modes', () => {
  // ConfigMission supports FEWER options than the catalog offers. The category
  // resolver must filter a ConfigMission robot's options to the supported subset,
  // and every option it DOES offer must be mappable by the encoder (no runtime throw).
  for (const param of ['mode_yaw', 'mode_trace', 'mode_landing']) {
    test(`${param}: ConfigMission robot only sees options the encoder can map`, () => {
      const options = categoryModel.getAtributesParam({ type: 'dji_M210_noetic', param });
      assert.ok(options.length > 0, `${param} should expose at least one option`);
      // resolve each offered option value → symbol → ConfigMission number (must not throw)
      for (const opt of options) {
        const symbol = categoryModel.symbolForValue(param, opt.id);
        assert.doesNotThrow(() => toConfigValue(param, symbol), `offered ${param}=${opt.id} not mappable`);
      }
    });
  }

  test('a PSDK robot sees MORE yaw options than a ConfigMission robot', () => {
    const psdk = categoryModel.getAtributesParam({ type: 'dji_M300_PSDK', param: 'mode_yaw' });
    const config = categoryModel.getAtributesParam({ type: 'dji_M210_noetic', param: 'mode_yaw' });
    assert.ok(psdk.length > config.length, 'PSDK should expose more yaw options');
  });

  test('toConfigValue throws on an unknown/unsupported symbol', () => {
    assert.throws(() => toConfigValue('mode_yaw', 'HEADING_MODE_POI'), { name: 'RangeError' });
  });
});

describe('profile derivation — serviceType + category, not a declared field', () => {
  test('PSDK serviceType derives the psdk profile', () => {
    assert.equal(profileFor('dji_M300_PSDK', 'psdk_interfaces/srv/InitWaypointV2Setting'), 'psdk');
  });

  test('ConfigMission serviceType derives the v2 profile by default', () => {
    // Unknown category on a ConfigMission service → base mapping is v2.
    assert.equal(profileFor('some_config_robot', 'aerialcore_common/ConfigMission'), 'v2');
    assert.equal(profileFor('some_config_robot', 'muav_gcs_interfaces/srv/LoadMission'), 'v2');
  });

  test('category override wins over the serviceType base mapping', () => {
    // M210 noetic / px4_mavros are pinned to v1 despite the ConfigMission service.
    assert.equal(profileFor('dji_M210_noetic', 'aerialcore_common/ConfigMission'), 'v1');
    assert.equal(profileFor('px4_mavros', 'aerialcore_common/ConfigMission'), 'v1');
    assert.equal(profileFor('dji_M600', 'aerialcore_common/ConfigMission'), 'default');
  });

  test('no serviceType (no mission service) → default', () => {
    assert.equal(profileFor('agv', undefined), 'default');
    assert.equal(profileFor('unknown', undefined), 'default');
  });

  test('derived profile is consistent with the encoder family (no Frankenstein)', () => {
    // psdk profile ⇒ PSDK encoder serviceType; never psdk profile on a ConfigMission service.
    assert.equal(profileFor('x', 'psdk_interfaces/srv/InitWaypointV2Setting'), 'psdk');
    assert.notEqual(profileFor('x', 'aerialcore_common/ConfigMission'), 'psdk');
  });
});

describe('default fallback — no device / no mission service', () => {
  // A serviceless category (agv) or an unknown one falls back to the bare 'default'
  // profile, so the planning UI always has a coherent (if minimal) baseline.
  const defaultParamIds = schema.profiles.default.params;
  for (const type of ['agv', 'noexiste', undefined]) {
    test(`'${type}' falls back to the default profile params`, () => {
      const attrs = categoryModel.getAttributesList(type);
      assert.deepEqual(
        attrs.map((a) => a.id),
        defaultParamIds,
        'should expose exactly the default profile params'
      );
    });
  }
});

// ─── Value-transform per family (the Anti-Corruption Layer) ───────────────────
// Same domain value (gimbal -90 deg) → DIFFERENT firmware encoding per family.
// This locks the boundary: PSDK ×10 (0.1° units), ConfigMission raw.

describe('per-waypoint params — mode_turn applies per waypoint', () => {
  test('getWaypointParams exposes mode_turn + speed for a PSDK robot', () => {
    const wp = categoryModel.getWaypointParams('dji_M300_PSDK').map((p) => p.id);
    assert.ok(wp.includes('mode_turn'), 'PSDK should expose per-waypoint mode_turn');
    assert.ok(wp.includes('speed'));
  });

  test('default profile exposes no per-waypoint params', () => {
    assert.equal(categoryModel.getWaypointParams('dji_M600').length, 0);
  });

  test('a profile without max_vel/mode_gimbal OMITS those keys from the message (not undefined)', () => {
    // v1 (M210) has no max_vel nor mode_gimbal — they must be absent, not sent as undefined.
    const v1 = encodeRosSrv({
      type: 'configureMission',
      msg: { uav: 'u', uav_type: 'dji_M210_noetic', attributes: {}, wp: [{ pos: [38, -3, 10] }] },
      msgType: 'aerialcore_common/ConfigMission',
    });
    assert.ok(!('maxVel' in v1), 'maxVel should be omitted on v1');
    assert.ok(!('gimbalPitchMode' in v1), 'gimbalPitchMode should be omitted on v1');
    // v2 (M300) does expose them.
    const v2 = encodeRosSrv({
      type: 'configureMission',
      msg: { uav: 'u', uav_type: 'dji_M300', attributes: {}, wp: [{ pos: [38, -3, 10] }] },
      msgType: 'aerialcore_common/ConfigMission',
    });
    assert.ok('maxVel' in v2 && 'gimbalPitchMode' in v2, 'v2 should include both');
  });

  test('each waypoint uses its own turn value, resolved via symbol to the PSDK number', () => {
    // wp[0] CW (catalog value 1 → symbol TURN_MODE_CLOCKWISE → PSDK 0)
    // wp[1] CCW (catalog value 2 → symbol TURN_MODE_COUNTER_CLOCKWISE → PSDK 1)
    const route = {
      uav: 'u',
      uav_type: 'dji_M300_PSDK',
      attributes: {},
      wp: [
        { pos: [38.1, -3.1, 10], mode_turn: 1 },
        { pos: [38.2, -3.2, 10], mode_turn: 2 },
      ],
    };
    const result = encodeRosSrv({
      type: 'configureMission',
      msg: route,
      msgType: 'psdk_interfaces/srv/InitWaypointV2Setting',
    });
    const mission = result.waypoint_v2_init_settings.mission;
    assert.equal(mission[0].turn_mode, toPsdkValue('mode_turn', 'TURN_MODE_CLOCKWISE'));
    assert.equal(mission[1].turn_mode, toPsdkValue('mode_turn', 'TURN_MODE_COUNTER_CLOCKWISE'));
    assert.notEqual(mission[0].turn_mode, mission[1].turn_mode); // genuinely per-waypoint
  });

  test('a waypoint without turn falls back to the catalog default', () => {
    const route = {
      uav: 'u',
      uav_type: 'dji_M300_PSDK',
      attributes: {},
      wp: [{ pos: [38.1, -3.1, 10] }], // no turn
    };
    const result = encodeRosSrv({
      type: 'configureMission',
      msg: route,
      msgType: 'psdk_interfaces/srv/InitWaypointV2Setting',
    });
    const defaultTurnSymbol = categoryModel.symbolForValue('mode_turn', schema.waypoint_params.mode_turn.default);
    assert.equal(result.waypoint_v2_init_settings.mission[0].turn_mode, toPsdkValue('mode_turn', defaultTurnSymbol));
  });
});

describe('encoder value-transform — gimbal -90 differs by family', () => {
  const wp = [{ pos: [38.1, -3.1, 10], action: { gimbal: -90 } }];

  test('PSDK encodes gimbal -90 as y = -900 (×10, 0.1° units)', () => {
    const route = { uav: 'u', uav_type: 'dji_M300_PSDK', attributes: {}, wp };
    const result = encodeRosSrv({
      type: 'configureMission',
      msg: route,
      msgType: 'psdk_interfaces/srv/InitWaypointV2Setting',
    });
    const gimbalAction = result.waypoint_v2_init_settings.actions.find(
      (a) => a.waypoint_v2_action_actuator_type === 2 // ACTUATOR.GIMBAL
    );
    assert.equal(gimbalAction.waypoint_v2_gimbal_actuator.waypoint_v2_gimbal_actuator_rotation_param.y, -900);
  });

  test('ConfigMission encodes gimbal -90 raw (no transform)', () => {
    // dji_M300 is v2 (exposes gimbal action) but uses the ConfigMission encoder.
    const route = { uav: 'u', uav_type: 'dji_M300', attributes: {}, wp };
    const result = encodeRosSrv({
      type: 'configureMission',
      msg: route,
      msgType: 'aerialcore_common/ConfigMission',
    });
    assert.ok(result.commandParameter.data.includes(-90), 'raw -90 should be present');
    assert.ok(!result.commandParameter.data.includes(-900), 'should NOT be ×10 transformed');
  });
});
