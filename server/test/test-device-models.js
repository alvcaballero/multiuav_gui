import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { deviceModelsDir } from '../config/config.js';
import { VEHICLE_TYPE } from '../config/status.js';
import {
  buildDeviceModelCatalog,
  defaultModelOf,
  genericModelOf,
  incompatibility,
  loadDeviceModels,
  mergeDeviceModel,
  modelInterfaceProblem,
} from '../models/deviceModels/deviceModelCatalog.js';
import { deviceModelsModel } from '../models/deviceModels/deviceModels.js';
import { categoryModel } from '../models/category.js';
import { buildCapabilityCatalog } from '../models/capabilities/capabilityCatalog.js';

// config/devices/device_models/: WHAT each vehicle is. A device = a device model + an interface
// (its category, HOW it is talked to); this file tests the models and which pairs work.

const generics = Object.fromEntries(Object.values(VEHICLE_TYPE).map((t) => [genericModelOf(t), { vehicleType: t, interfaces: [], capabilities: '*' }]));
const problemsOf = (raw) => buildDeviceModelCatalog({ ...generics, ...raw }).problems;

describe('the real device model files', () => {
  const { catalog, problems } = loadDeviceModels(deviceModelsDir);

  test('every file validates', () => {
    assert.deepEqual(problems, []);
  });

  test('every interface has a default model that exists and lists it', () => {
    for (const [key, iface] of Object.entries(categoryModel.getCatalog())) {
      const modelKey = defaultModelOf(iface);
      assert.ok(catalog[modelKey], `${key}: ${modelKey}`);
      assert.equal(modelInterfaceProblem(modelKey, catalog[modelKey], key, iface), null, key);
    }
  });

  test('a DJI model inherits the shared rotorcraft numbers and overrides only what differs', () => {
    assert.equal(catalog.dji_M600.physics.number_of_rotors, 6);
    assert.equal(catalog.dji_M600.physics.mass, catalog.dji_M210.physics.mass);
    // The M300 adds the side and back sensors to the inherited down/front/up.
    assert.deepEqual(Object.keys(catalog.dji_M300.obstacle_sensor).sort(), ['back', 'down', 'front', 'left', 'right', 'up']);
    assert.equal(catalog._dji_rotorcraft, undefined); // abstract: only a base
  });

  test('the firmware a vehicle runs is a fact of its model', () => {
    assert.deepEqual(catalog.griffin.firmware, ['ardupilot']);
    assert.deepEqual(catalog.fuvex.firmware, ['px4']);
    assert.deepEqual(catalog.catec.firmware, ['px4']);
    assert.deepEqual(catalog.x500.firmware, ['px4', 'ardupilot']);
    assert.equal(catalog.generic_multirotor.firmware, undefined); // not known: any
  });
});

describe('buildDeviceModelCatalog', () => {
  test('every vehicle type needs its generic model', () => {
    const { problems } = buildDeviceModelCatalog({ generic_multirotor: { vehicleType: 'multirotor', interfaces: [], capabilities: '*' } });
    assert.ok(problems.some((p) => p.path[0] === 'generic_usv'));
  });

  test('a typo, a bad sensor range or a model without vehicle type, interfaces or capabilities is an error', () => {
    const ok = { vehicleType: 'multirotor', interfaces: [], capabilities: {} };
    assert.equal(problemsOf({ m: { ...ok, phisics: {} } }).length, 1);
    assert.equal(problemsOf({ m: { ...ok, obstacle_sensor: { down: [0] } } }).length, 1);
    assert.deepEqual(problemsOf({ m: { interfaces: [], capabilities: {} } }), [
      { path: ['m', 'vehicleType'], message: 'missing vehicleType' },
    ]);
    assert.deepEqual(problemsOf({ m: { vehicleType: 'multirotor', capabilities: {} } }), [
      { path: ['m', 'interfaces'], message: 'missing interfaces (the list may be empty)' },
    ]);
    assert.deepEqual(problemsOf({ m: { vehicleType: 'multirotor', interfaces: [] } }), [
      { path: ['m', 'capabilities'], message: "missing capabilities ({} for none, '*' for a generic)" },
    ]);
    assert.equal(problemsOf({ m: { ...ok, interfaces: ['a', 'a'] } }).length, 1);
    assert.equal(problemsOf({ m: { ...ok, capabilities: { rtk: 'sometimes' } } }).length, 1);
  });

  test('extends merges objects key by key and replaces the rest', () => {
    const parent = { vehicleType: 'multirotor', firmware: ['px4'], physics: { mass: 2, rotor_blades: 2 } };
    assert.deepEqual(mergeDeviceModel(parent, { firmware: ['ardupilot'], physics: { mass: 3 } }), {
      vehicleType: 'multirotor',
      firmware: ['ardupilot'],
      physics: { mass: 3, rotor_blades: 2 },
    });
  });
});

describe('model ↔ interface', () => {
  const griffin = { vehicleType: 'flapping_wing', firmware: ['ardupilot'] };
  const mavros = { vehicleTypes: ['flapping_wing'], firmwares: ['px4', 'ardupilot'] };

  test('compatible when the interface serves its vehicle type and drives one of its firmwares', () => {
    assert.equal(incompatibility('griffin', griffin, 'griffin', mavros), null);
  });

  test('a vehicle type the interface does not serve', () => {
    assert.equal(
      incompatibility('griffin', griffin, 'ardupilot_mavlink', { vehicleTypes: ['multirotor'], firmwares: ['ardupilot'] }),
      "model 'griffin' is a flapping_wing, and interface 'ardupilot_mavlink' serves multirotor"
    );
  });

  test('a firmware the interface does not drive', () => {
    assert.equal(
      incompatibility('dji_M300', { vehicleType: 'multirotor', firmware: ['dji'] }, 'px4_mavlink', { vehicleTypes: ['multirotor'], firmwares: ['px4'] }),
      "model 'dji_M300' runs dji, and interface 'px4_mavlink' drives px4"
    );
  });

  test('a model that declares no firmware fits any interface of its vehicle type', () => {
    assert.equal(incompatibility('generic_multirotor', { vehicleType: 'multirotor' }, 'x', { vehicleTypes: ['multirotor'], firmwares: ['dji'] }), null);
  });
});

describe('deviceModelsModel', () => {
  test('every model and every interface it lists loaded: the boot check found nothing wrong', () => {
    const { catalog } = loadDeviceModels(deviceModelsDir);
    assert.deepEqual(Object.keys(deviceModelsModel.getAll()).sort(), Object.keys(catalog).sort());
    for (const [key, model] of Object.entries(catalog)) {
      for (const iface of model.interfaces) {
        assert.equal(incompatibility(key, model, iface, categoryModel.getCategory(iface)), null, `${key} → ${iface}`);
      }
    }
  });

  test('a model offers exactly the interfaces it lists: x500 never sees external_telemetry (the CATEC/FuVeX topics)', () => {
    assert.deepEqual(deviceModelsModel.interfacesOf('x500'), ['px4_ros2', 'px4_mavros', 'px4_mavlink', 'ardupilot_mavlink']);
    assert.deepEqual(deviceModelsModel.interfacesOf('catec'), ['external_telemetry']);
    assert.deepEqual(deviceModelsModel.interfacesOf('nope'), []);
  });

  test('list({ category }): the models that list it, its default first', () => {
    const names = deviceModelsModel.list({ category: 'dji_M300_PSDK' }).map((m) => m.name);
    assert.deepEqual(names, ['dji_M300']);
    assert.deepEqual(
      deviceModelsModel.list({ category: 'px4_mavros' }).map((m) => m.name),
      ['x500', 'generic_multirotor', 'standard_vtol', 'generic_vtol']
    );
  });

  test('the VTOL fits px4_mavros (vtol_1, with the slow relay as a device attribute now)', () => {
    assert.equal(deviceModelsModel.deviceProblem({ category: 'px4_mavros', deviceModel: 'standard_vtol' }), null);
  });

  test('defaultFor: the interface default, or the generic of its first vehicle type', () => {
    assert.equal(deviceModelsModel.defaultFor('dji_M210_melodic'), 'dji_M210'); // inherited from noetic
    assert.equal(deviceModelsModel.defaultFor('px4_ros2'), 'x500');
    assert.equal(deviceModelsModel.defaultFor('nope'), null);
  });

  test('deviceProblem: the interface must be one the model lists (the form picks the model first)', () => {
    assert.deepEqual(deviceModelsModel.deviceProblem({ category: 'external_telemetry', deviceModel: 'x500' }), {
      path: ['category'],
      message:
        "model 'x500' does not list interface 'external_telemetry' (its interfaces: px4_ros2, px4_mavros, px4_mavlink, ardupilot_mavlink)",
    });
    assert.deepEqual(deviceModelsModel.deviceProblem({ category: 'agv', deviceModel: 'nope' }), {
      path: ['deviceModel'],
      message: "unknown device model 'nope'",
    });
  });

  test('platformOf: what the client draws, with the generic of its type filling what the model leaves out', () => {
    assert.deepEqual(deviceModelsModel.platformOf('dji_M300'), {
      vehicleType: 'multirotor',
      label: 'DJI Matrice 300',
      icon: 'dji_M300',
      model3d: 'm300',
    });
    assert.equal(deviceModelsModel.platformOf('bcr_bot').icon, 'default'); // from generic_ugv
    assert.equal(deviceModelsModel.platformOf(null), null);
  });

  test('planningProfileOf: what the planners are told, only the blocks the model declares', () => {
    const vtol = deviceModelsModel.planningProfileOf({
      deviceModel: 'standard_vtol',
      category: 'px4_mavlink',
      attributes: {},
    });
    assert.equal(vtol.vehicleType, 'vtol');
    assert.deepEqual(vtol.firmware, ['px4']);
    // Effective: the model declares file_download, MAVLink does not implement it
    assert.deepEqual(vtol.capabilities, ['autonomous_mission', 'flight_control']);
    // The same numbers the mission validator warns with
    assert.equal(vtol.speeds.fixed_wing, 15);
    assert.equal(vtol.limits.min_turn_radius, 19.2);
    assert.equal(vtol.vtol.transition_airspeed, 10);
    // Not declared: left out for the planner to use its own, never a default
    assert.equal('physics' in vtol, false);
    assert.equal('battery' in vtol, false);

    const m300 = deviceModelsModel.planningProfileOf({ deviceModel: 'dji_M300', category: 'dji_M300', attributes: {} });
    assert.equal(m300.label, 'DJI Matrice 300');
    assert.equal(m300.physics.mass, 7.2); // inherited from _dji_rotorcraft
    assert.deepEqual(Object.keys(m300.obstacle_sensor).sort(), ['back', 'down', 'front', 'left', 'right', 'up']);
  });

  test('planningProfileOf: an unknown model gives only its capabilities', () => {
    const profile = deviceModelsModel.planningProfileOf({ deviceModel: 'nope', category: 'px4_ros2', attributes: {} });
    assert.deepEqual(Object.keys(profile).sort(), ['capabilities', 'deviceModel', 'vehicleType']);
    assert.equal(profile.vehicleType, null);
    assert.equal(deviceModelsModel.planningProfileOf({ deviceModel: 'x500', category: 'nope' }).capabilities, null);
  });
});

describe('capabilities', () => {
  const dev = (deviceModel, category, capabilities) => ({ deviceModel, category, attributes: capabilities ? { capabilities } : {} });
  const caps = (d) => [...deviceModelsModel.capabilitiesOf(d)].sort();

  test('a device has its model capabilities that its interface implements', () => {
    assert.deepEqual(caps(dev('dji_M300', 'dji_M300')), [
      'autonomous_mission',
      'camera',
      'camera_stream',
      'file_download',
      'gimbal',
      'obstacle_sensing',
    ]);
    assert.deepEqual(caps(dev('dji_M600', 'dji_M600')), []); // telemetry only
    assert.deepEqual(caps(dev('catec', 'external_telemetry')), ['threat_detection']);
    assert.deepEqual(caps(dev('fuvex', 'external_telemetry')), []);
  });

  test('griffin on ardupilot_mavlink is telemetry only: the interface has Takeoff, griffin does not fly on command', () => {
    const griffin = dev('griffin', 'ardupilot_mavlink');
    assert.deepEqual(caps(griffin), []);
    assert.ok(!deviceModelsModel.allowsCommand(griffin, 'Takeoff'));
    assert.ok(!deviceModelsModel.allowsCommand(griffin, 'LoadMission'));
    assert.ok(deviceModelsModel.allowsCommand(griffin, 'SaveHome')); // no capability owns it
    assert.ok(deviceModelsModel.allowsSlot(griffin, 'position_global'));
  });

  test('griffin on its mavros interface reads the humidity sensor; an x500 there would not', () => {
    assert.ok(deviceModelsModel.allowsSlot(dev('griffin', 'griffin'), 'humidity'));
    assert.ok(!deviceModelsModel.allowsSlot(dev('x500', 'griffin'), 'humidity'));
  });

  test('an optional capability is off until the device enables it; a standard one off when it disables it', () => {
    assert.ok(!deviceModelsModel.allowsSlot(dev('dji_M210', 'dji_M210_noetic'), 'position_rtk'));
    assert.ok(deviceModelsModel.allowsSlot(dev('dji_M210', 'dji_M210_noetic', { rtk: true }), 'position_rtk'));
    assert.ok(!deviceModelsModel.allowsCommand(dev('dji_M210', 'dji_M210_noetic', { gimbal: false }), 'Gimbal'));
  });

  test('a generic model has everything its interface implements, as before device models', () => {
    const generic = dev('generic_multirotor', 'px4_ros2');
    for (const command of ['Takeoff', 'LoadMission', 'Gimbal']) assert.ok(deviceModelsModel.allowsCommand(generic, command), command);
  });

  test('an unknown category gates nothing (the boot reports that device)', () => {
    assert.equal(deviceModelsModel.capabilitiesOf(dev('x500', 'nope')), null);
    assert.ok(deviceModelsModel.allowsCommand(dev('x500', 'nope'), 'Takeoff'));
  });

  test('a device may only switch the capabilities its model declares', () => {
    assert.equal(deviceModelsModel.capabilityOverrideProblem(dev('dji_M210', 'dji_M210_noetic', { rtk: true })), null);
    assert.deepEqual(deviceModelsModel.capabilityOverrideProblem(dev('fuvex', 'external_telemetry', { threat_detection: true })), {
      path: ['attributes', 'capabilities', 'threat_detection'],
      message: "model 'fuvex' does not declare capability 'threat_detection' (it declares: none)",
    });
  });

  test('the catalog: each command and slot belongs to at most one capability, and they all exist', () => {
    assert.ok(Object.keys(deviceModelsModel.getCapabilities()).length > 0);
    assert.deepEqual(buildCapabilityCatalog({ a: { label: 'A', commands: ['Takeoff'] }, b: { label: 'B', commands: ['Takeoff'] } }).problems, [
      { path: ['b'], message: "'Takeoff' already belongs to 'a'" },
    ]);
    assert.equal(buildCapabilityCatalog({ a: { label: 'A', commands: ['Fly'] } }).problems.length, 1);
    assert.equal(buildCapabilityCatalog({ a: { label: 'A', slots: ['nope'] } }).problems.length, 1);
  });

  test('the catalog: mission items exist in mission_schema.yaml and belong to at most one capability', () => {
    const keys = categoryModel.missionKeys();
    const twice = { a: { label: 'A', mission: { actions: ['photo'] } }, b: { label: 'B', mission: { actions: ['photo'] } } };
    assert.deepEqual(buildCapabilityCatalog(twice, keys).problems, [{ path: ['b'], message: "'actions:photo' already belongs to 'a'" }]);
    assert.deepEqual(buildCapabilityCatalog({ a: { label: 'A', mission: { params: ['nope'] } } }, keys).problems, [
      { path: ['a', 'mission', 'params'], message: "'nope' is not in mission_schema.yaml params" },
    ]);
  });
});

describe('capabilities: mission content', () => {
  const dev = (deviceModel, category, capabilities) => ({ deviceModel, category, attributes: capabilities ? { capabilities } : {} });
  const actionsOf = (d) => categoryModel.getActions({ type: d.category }, deviceModelsModel.missionRestrictionOf(d)).map((a) => a.name);

  // camera has no command nor slot: only its mission actions say an interface implements it.
  test('an interface implements a capability through its mission profile', () => {
    assert.ok(deviceModelsModel.capabilitiesOf(dev('x500', 'px4_mavlink', { camera: true })).has('camera'));
    assert.ok(!deviceModelsModel.capabilitiesOf(dev('x500', 'px4_mavlink')).has('camera'), 'optional: off until enabled');
  });

  test('one interface, two vehicles: each sees only what it can do', () => {
    const vtol = actionsOf(dev('standard_vtol', 'px4_mavlink'));
    const x500 = actionsOf(dev('x500', 'px4_mavlink'));
    assert.ok(vtol.includes('vtol_transition') && !x500.includes('vtol_transition'), 'vtol_transition only for a VTOL');
    assert.ok(!vtol.includes('gimbal') && !x500.includes('gimbal'), 'no gimbal (x500: optional and off)');
    assert.ok(!x500.includes('photo'), 'no camera until enabled');
    assert.ok(actionsOf(dev('x500', 'px4_mavlink', { camera: true, gimbal: true })).includes('photo'));
    assert.ok(x500.includes('stay') && x500.includes('yaw'), 'what no capability owns, every device has');
  });

  test('a route param and a waypoint param owned by a capability go with it', () => {
    const restrict = (d) => deviceModelsModel.missionRestrictionOf(d);
    const ids = (list) => list.map((p) => p.id);
    const m300 = dev('dji_M300', 'dji_M300'); // ConfigMission v2: has mode_gimbal and the wp gimbal
    assert.ok(ids(categoryModel.getWaypointParams(m300.category, restrict(m300))).includes('gimbal'));
    assert.ok(ids(categoryModel.getAttributesList(m300.category, restrict(m300))).includes('mode_gimbal'));
    const noGimbal = dev('dji_M300', 'dji_M300', { gimbal: false }); // a broken gimbal
    assert.ok(!ids(categoryModel.getWaypointParams(noGimbal.category, restrict(noGimbal))).includes('gimbal'));
    assert.ok(!ids(categoryModel.getAttributesList(noGimbal.category, restrict(noGimbal))).includes('mode_gimbal'));
  });

  test('without a device the profile is what the interface encodes, as before', () => {
    const all = categoryModel.getActions({ type: 'px4_mavlink' }).map((a) => a.name);
    for (const action of ['vtol_transition', 'gimbal', 'photo']) assert.ok(all.includes(action), action);
  });
});
