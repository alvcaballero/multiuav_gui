import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { readDataFile } from '../common/utils.js';
import { devicesMsg } from '../config/config.js';
import { CommandType } from '../config/commandTypes.js';
import { commandRegistry, missionTaskForDevice } from '../models/commands/commandRegistry.js';
import { buildVehicleCommand } from '../models/commands/vehicleCommand.js';
import { hasMsgEncoder } from '../models/msgEncode.js';

const devices = readDataFile(devicesMsg);
// Golden: los commands que expone cada categoría. Se generó con el catálogo anterior
// (`requires` por key), así el registry prueba que deriva EXACTAMENTE lo mismo. Si agregás un
// endpoint o un encoder que cambia lo que expone un dron, el diff de este archivo lo muestra.
const golden = JSON.parse(readFileSync(new URL('./api/json/category_commands.json', import.meta.url)));

const POS = { latitude: 37.41, longitude: -6.0, altitude: 50, course: 0, attributes: { gimbal: [-30, 0, 15] } };
const endpoint = (block, key, msgType) => ({ block, key, msgType });

describe('commandRegistry — what each category exposes', () => {
  test('the golden file covers every category of devices_msg.yaml', () => {
    assert.deepEqual(Object.keys(golden).sort(), Object.keys(devices).sort());
  });

  for (const [category, expected] of Object.entries(golden)) {
    test(`${category}`, () => {
      const { commands, problems } = commandRegistry.resolveCommands(devices[category]);
      assert.deepEqual(problems, []);
      assert.deepEqual([...commands.keys()].sort(), expected);
    });
  }

  test('every endpoint a command goes out by has a message encoder (or an explicit passthrough)', () => {
    const unencoded = Object.entries(devices).flatMap(([category, def]) =>
      [...commandRegistry.resolveCommands(def).commands]
        .filter(([, endpoint]) => endpoint && !hasMsgEncoder(endpoint.msgType))
        .map(([type, endpoint]) => `${category}.${type} (${endpoint.msgType})`)
    );
    assert.deepEqual(unencoded, []);
  });

  test('every CommandType has an encoder or is local', () => {
    const everything = {
      services: Object.fromEntries(Object.values(devices).flatMap((d) => Object.entries(d?.services ?? {}))),
      actions: Object.fromEntries(Object.values(devices).flatMap((d) => Object.entries(d?.actions ?? {}))),
      publishers: Object.fromEntries(Object.values(devices).flatMap((d) => Object.entries(d?.publishers ?? {}))),
    };
    // Una categoría ficticia con un endpoint de cada tipo: lo que quede afuera no tiene encoder.
    const covered = new Set();
    for (const [block, entries] of Object.entries(everything)) {
      for (const [key, def] of Object.entries(entries)) {
        const { commands } = commandRegistry.resolveCommands({ [block]: { [key]: def } });
        for (const type of commands.keys()) covered.add(type);
      }
    }
    assert.deepEqual(Object.values(CommandType).filter((t) => !covered.has(t)), []);
  });
});

describe('commandRegistry — encoders build what the gateway gets', () => {
  test('flight commands build the PX4 VehicleCommand', () => {
    const ep = endpoint('publishers', 'vehicle_command', 'px4_msgs/msg/VehicleCommand');
    const attributes = { altitude: 20 };
    assert.deepEqual(
      commandRegistry.encode(CommandType.TAKEOFF, ep, { attributes, position: POS }),
      buildVehicleCommand({ type: CommandType.TAKEOFF, attributes, position: POS })
    );
  });

  test('gimbal fills the missing axes from telemetry; reset forces the flag', () => {
    const osdk = endpoint('services', 'gimbal_control', 'dji_osdk_ros/GimbalAction');
    const psdk = endpoint('publishers', 'gimbal_control', 'psdk_interfaces/msg/GimbalRotation');
    const ctx = { attributes: { pitch: -60 }, position: POS };
    const expected = { reset: false, pitch: -60, roll: 0, yaw: 15 };
    assert.deepEqual(commandRegistry.encode(CommandType.GIMBAL, osdk, ctx), expected);
    assert.deepEqual(commandRegistry.encode(CommandType.GIMBAL_PITCH, psdk, ctx), expected);
    assert.equal(commandRegistry.encode(CommandType.RESET_GIMBAL, osdk, { position: POS }).reset, true);
  });

  test('command_mission sends data: true; no-argument services send no request', () => {
    const start = endpoint('services', 'command_mission', 'std_srvs/SetBool');
    assert.deepEqual(commandRegistry.encode(CommandType.COMMAND_MISSION, start), { data: true });
    const stop = endpoint('services', 'stop_mission', 'std_srvs/Trigger');
    assert.equal(commandRegistry.encode(CommandType.STOP_MISSION, stop), undefined);
  });

  test('attribute services pass the validated attributes through', () => {
    const ep = endpoint('services', 'setup_camera', 'dji_osdk_ros/SetupCameraStream');
    const attributes = { cameraType: 1, start: 1 };
    assert.equal(commandRegistry.encode(CommandType.SETUP_CAMERA, ep, { attributes }), attributes);
  });

  test('SaveHome is local, with no endpoint', () => {
    assert.equal(typeof commandRegistry.localRun(CommandType.SAVE_HOME), 'function');
  });
});

describe('missionTaskForDevice', () => {
  const device = { name: 'uav1', category: 'px4_ros2' };
  const wp = [{ pos: [0, 0, 10] }];

  test('picks the device task and tags it with the category', () => {
    const mission = { tasks: [{ task_id: 'T1', device: 'uav1', depends_on: [], params: {}, wp }] };
    assert.deepEqual(missionTaskForDevice(mission, device), { ...mission.tasks[0], uav_type: 'px4_ros2' });
  });

  test('accepts the bare route array the MCP posts', () => {
    assert.equal(missionTaskForDevice([{ uav: 'uav1', wp }], device).device, 'uav1');
  });

  test('rejects an empty mission, a mission without the device, and an ambiguous one', () => {
    assert.throws(() => missionTaskForDevice([], device), /no mission/);
    assert.throws(() => missionTaskForDevice([{ uav: 'other', wp }], device), /has no task/);
    const twice = {
      tasks: [
        { task_id: 'T1', device: 'uav1', depends_on: [], params: {}, wp },
        { task_id: 'T2', device: 'uav1', depends_on: ['T1'], params: {}, wp },
      ],
    };
    assert.throws(() => missionTaskForDevice(twice, device), /several tasks \(T1, T2\)/);
  });
});
