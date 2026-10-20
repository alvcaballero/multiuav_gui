import { test, describe, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { commandsModel } from '../models/commands/commands.js';
import { registerGateway } from '../models/gateways/registry.js';
import { devicesController } from '../controllers/devices.js';
import { positionsController } from '../controllers/positions.js';
import { eventsController } from '../controllers/events.js';
import { CommandType } from '../config/commandTypes.js';

// sendCommand de punta a punta, sin ROS: un gateway de prueba registra qué endpoint y qué
// mensaje le llegan. Las categorías son las reales de devices_msg.yaml.
const DEVICES = {
  1: { id: 1, name: 'px4_1', category: 'px4_ros2', protocol: 'test' },
  2: { id: 2, name: 'm300_1', category: 'dji_M300_PSDK', protocol: 'test' },
};
const POSITION = {
  latitude: 37.41,
  longitude: -6.0,
  altitude: 50,
  course: 0,
  attributes: { gimbal: [-30, 0, 15], systemId: 1 },
};

let sent = [];
let homes = [];
registerGateway({ protocol: 'test', sendCommand: async (call) => (sent.push(call), { state: 'success', msg: 'sent' }) });
devicesController.getDevice = async (id) => DEVICES[id];
positionsController.getByDeviceId = () => POSITION;
positionsController.updatePosition = (payload) => homes.push(payload);
eventsController.addEvent = async () => {};

beforeEach(() => {
  sent = [];
  homes = [];
});

describe('sendCommand', () => {
  test('a PX4 flight command goes out as a VehicleCommand on the publisher', async () => {
    const r = await commandsModel.sendCommand({ deviceId: 1, type: CommandType.TAKEOFF, attributes: { altitude: '20' } });
    assert.equal(r.state, 'success');
    assert.deepEqual(sent[0].endpoint, { block: 'publishers', key: 'vehicle_command', msgType: 'px4_msgs/msg/VehicleCommand' });
    assert.equal(sent[0].message.command, 22);
    assert.equal(sent[0].message.param7, 70);
    assert.equal(sent[0].message.target_system, 1);
  });

  test('a PX4 Gimbal fills the axis that does not come with the telemetry, like DJI', async () => {
    await commandsModel.sendCommand({ deviceId: 1, type: CommandType.GIMBAL, attributes: { pitch: -60 } });
    assert.equal(sent[0].endpoint.key, 'vehicle_command');
    assert.equal(sent[0].message.command, 1000); // DO_GIMBAL_MANAGER_PITCHYAW
    assert.equal(sent[0].message.param1, -60); // pitch, degrees
    assert.equal(sent[0].message.param2, 15); // yaw kept: attributes.gimbal = [-30, 0, 15]
  });

  test('a PX4 command is not sent while the vehicle has not reported its system id', async () => {
    positionsController.getByDeviceId = () => ({ ...POSITION, attributes: {} });
    try {
      const r = await commandsModel.sendCommand({ deviceId: 1, type: CommandType.LAND });
      assert.equal(r.state, 'error');
      assert.match(r.msg, /system id/);
      assert.equal(sent.length, 0);
    } finally {
      positionsController.getByDeviceId = () => POSITION;
    }
  });

  test('the same Gimbal command goes out by the PSDK publisher, already encoded for its msgType', async () => {
    await commandsModel.sendCommand({ deviceId: 2, type: CommandType.GIMBAL, attributes: { pitch: -60 } });
    assert.equal(sent[0].endpoint.block, 'publishers');
    assert.equal(sent[0].endpoint.msgType, 'psdk_interfaces/msg/GimbalRotation');
    const rad = (deg) => (deg * Math.PI) / 180;
    // pitch from the command, roll/yaw from telemetry; absolute mode, radians.
    assert.deepEqual(sent[0].message, {
      payload_index: 1,
      rotation_mode: 1,
      pitch: rad(-60),
      roll: rad(0),
      yaw: rad(15),
      time: 0,
    });
  });

  test('CommandMission goes to its service with data: true', async () => {
    await commandsModel.sendCommand({ deviceId: 1, type: CommandType.COMMAND_MISSION });
    assert.equal(sent[0].endpoint.key, 'command_mission');
    assert.deepEqual(sent[0].message, { data: true });
  });

  test('LoadMission sends the device task; a mission without it is an error', async () => {
    const wp = [{ pos: [0, 0, 10] }];
    await commandsModel.sendCommand({ deviceId: 1, type: CommandType.LOAD_MISSION, attributes: [{ uav: 'px4_1', wp }] });
    assert.equal(sent[0].endpoint.key, 'configure_mission');
    // px4_ros2 declares muav_gcs_interfaces/srv/LoadMission: the ROS2 request, not the task.
    assert.equal(sent[0].endpoint.msgType, 'muav_gcs_interfaces/srv/LoadMission');
    assert.equal(sent[0].message.request.type, 'waypoint');
    assert.equal(sent[0].message.request.waypoint.length, 1);

    const r = await commandsModel.sendCommand({ deviceId: 1, type: CommandType.LOAD_MISSION, attributes: [{ uav: 'other', wp }] });
    assert.equal(r.state, 'error');
    assert.match(r.msg, /has no task/);
    assert.equal(sent.length, 1);
  });

  test('SaveHome runs locally, without a gateway', async () => {
    const r = await commandsModel.sendCommand({ deviceId: 1, type: CommandType.SAVE_HOME });
    assert.equal(r.state, 'success');
    assert.deepEqual(homes, [{ deviceId: 1, setHome: true }]);
    assert.equal(sent.length, 0);
  });

  test('rejects before sending: unknown type, unsupported command, invalid attributes', async () => {
    assert.match((await commandsModel.sendCommand({ deviceId: 1, type: 'Fly' })).msg, /does not exist/);
    assert.match((await commandsModel.sendCommand({ deviceId: 1, type: CommandType.SETUP_CAMERA })).msg, /not supported by device px4_1/);
    const bad = await commandsModel.sendCommand({ deviceId: 1, type: CommandType.TAKEOFF, attributes: { altitude: 'high' } });
    assert.match(bad.msg, /'altitude' must be a number/);
    assert.equal(sent.length, 0);
  });
});

describe('getCommandTypes', () => {
  test('each command says which endpoint implements it', async () => {
    const types = await commandsModel.getCommandTypes(1);
    const takeoff = types.find((t) => t.type === CommandType.TAKEOFF);
    assert.equal(takeoff.via, 'publishers.vehicle_command');
    assert.equal(takeoff.requires, 'vehicle_command');
    assert.equal(takeoff.label, 'Take off');
    const home = types.find((t) => t.type === CommandType.SAVE_HOME);
    assert.equal(home.via, null);
  });
});
