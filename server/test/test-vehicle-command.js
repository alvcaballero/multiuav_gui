import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { buildVehicleCommand, relativeTarget, VEHICLE_CMD } from '../models/vehicleCommand.js';
import { CommandType } from '../config/commandTypes.js';
import { commandRegistry } from '../models/commandRegistry.js';
import { approxDistanceMeters } from '../common/geo.js';
import { commandSchema, resolveCommand, validateAttributes } from '../models/commandSchema.js';

const POS = { latitude: 37.41, longitude: -6.0, altitude: 50, course: 0 };
const close = (a, b, eps = 1e-6) => assert.ok(Math.abs(a - b) < eps, `${a} != ${b}`);

describe('relativeTarget (body frame → global)', () => {
  test('forward with heading north moves north', () => {
    const t = relativeTarget(POS, { dx: 10 });
    assert.ok(t.latitude > POS.latitude);
    close(t.longitude, POS.longitude);
    close(approxDistanceMeters(POS.latitude, POS.longitude, t.latitude, t.longitude), 10, 0.01);
  });

  test('forward with heading east moves east, right moves south', () => {
    const east = { ...POS, course: 90 };
    const fwd = relativeTarget(east, { dx: 10 });
    assert.ok(fwd.longitude > POS.longitude);
    close(fwd.latitude, POS.latitude, 1e-9);
    const right = relativeTarget(east, { dy: 10 });
    assert.ok(right.latitude < POS.latitude);
  });

  test('dz adds to altitude and dyaw rotates clockwise, in radians', () => {
    const t = relativeTarget({ ...POS, course: 170 }, { dz: -5, dyaw: 20 });
    assert.equal(t.altitude, 45);
    close(t.yaw, (-170 * Math.PI) / 180); // 190° wraps to -170°
  });
});

describe('buildVehicleCommand', () => {
  test('every command the registry implements with a PX4 VehicleCommand builds a message', () => {
    const endpoint = { block: 'publishers', key: 'VehicleCommand', msgType: 'px4_msgs/msg/VehicleCommand' };
    const types = Object.values(CommandType).filter((type) => commandRegistry.encoderFor(type, endpoint));
    assert.ok(types.length > 0);
    for (const type of types) {
      // What sendCommand hands the builder: the schema defaults of an empty send.
      const checked = validateAttributes(resolveCommand(commandSchema, type), {});
      assert.ok(checked.ok, checked.msg);
      const msg = buildVehicleCommand({ type, attributes: checked.values, position: POS });
      assert.equal(typeof msg.command, 'number', type);
      // rosbridge carries JSON: a NaN would arrive as null.
      for (const [k, v] of Object.entries(msg)) assert.ok(!Number.isNaN(v), `${type}.${k} is NaN`);
    }
  });

  test('takeoff targets current altitude + requested height', () => {
    const msg = buildVehicleCommand({ type: CommandType.TAKEOFF, attributes: { altitude: 20 }, position: POS });
    assert.equal(msg.command, VEHICLE_CMD.NAV_TAKEOFF);
    assert.equal(msg.param7, 70);
    assert.equal(msg.param5, POS.latitude);
  });

  test('kill motors is a forced disarm', () => {
    const msg = buildVehicleCommand({ type: CommandType.KILL_MOTORS });
    assert.equal(msg.command, VEHICLE_CMD.COMPONENT_ARM_DISARM);
    assert.equal(msg.param1, 0);
    assert.equal(msg.param2, 21196);
  });

  test('moveRelative is a DO_REPOSITION to the absolute target', () => {
    const msg = buildVehicleCommand({
      type: CommandType.MOVE_RELATIVE,
      attributes: { dx: 5, dz: 3 },
      position: POS,
    });
    assert.equal(msg.command, VEHICLE_CMD.DO_REPOSITION);
    assert.equal(msg.param7, 53);
    assert.ok(msg.param5 > POS.latitude);
  });

  test('position-dependent commands fail without a position', () => {
    assert.throws(() => buildVehicleCommand({ type: CommandType.MOVE_RELATIVE, attributes: { dx: 1 } }), /position/);
    assert.throws(() => buildVehicleCommand({ type: CommandType.TAKEOFF }), /position/);
  });

  test('takeoff with the schema default climbs 10 m', () => {
    const { values } = validateAttributes(resolveCommand(commandSchema, CommandType.TAKEOFF), { altitude: '' });
    const msg = buildVehicleCommand({ type: CommandType.TAKEOFF, attributes: values, position: POS });
    assert.equal(msg.param7, 60);
  });

  test('a non vehicle command is rejected', () => {
    assert.throws(() => buildVehicleCommand({ type: CommandType.GIMBAL }), /not a vehicle command/);
  });
});
