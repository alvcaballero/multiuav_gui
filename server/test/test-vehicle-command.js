import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { buildVehicleCommand, buildGimbalTakeControl, relativeTarget, VEHICLE_CMD } from '../models/commands/vehicleCommand.js';
import { CommandType } from '../config/commandTypes.js';
import { commandRegistry } from '../models/commands/commandRegistry.js';
import { approxDistanceMeters } from '../common/geo.js';
import { commandSchema, resolveCommand, validateAttributes } from '../models/commands/commandSchema.js';

const POS = { latitude: 37.41, longitude: -6.0, altitude: 50, course: 0, attributes: { systemId: 3 } };
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

const NAN_ON_PURPOSE = { [CommandType.GIMBAL]: ['param3', 'param4'] }; // gimbal rates

describe('buildVehicleCommand', () => {
  test('every command the registry implements with a PX4 VehicleCommand builds a message', () => {
    const endpoint = { block: 'publishers', key: 'vehicle_command', msgType: 'px4_msgs/msg/VehicleCommand' };
    const types = Object.values(CommandType).filter((type) => commandRegistry.encoderFor(type, endpoint));
    assert.ok(types.length > 0);
    for (const type of types) {
      // What sendCommand hands the builder: the schema defaults of an empty send.
      const checked = validateAttributes(resolveCommand(commandSchema, type), {});
      assert.ok(checked.ok, checked.msg);
      const msg = buildVehicleCommand({ type, attributes: checked.values, position: POS });
      assert.equal(typeof msg.command, 'number', type);
      // NaN only where PX4 reads it as "unset" on purpose: everything else is filled from telemetry.
      const nanAllowed = NAN_ON_PURPOSE[type] ?? [];
      for (const [k, v] of Object.entries(msg)) {
        if (!nanAllowed.includes(k)) assert.ok(!Number.isNaN(v), `${type}.${k} is NaN`);
      }
    }
  });

  test('takeoff targets current altitude + requested height', () => {
    const msg = buildVehicleCommand({ type: CommandType.TAKEOFF, attributes: { altitude: 20 }, position: POS });
    assert.equal(msg.command, VEHICLE_CMD.VEHICLE_CMD_NAV_TAKEOFF);
    assert.equal(msg.param7, 70);
    assert.equal(msg.param5, POS.latitude);
  });

  test('kill motors is a forced disarm', () => {
    const msg = buildVehicleCommand({ type: CommandType.KILL_MOTORS, position: POS });
    assert.equal(msg.command, VEHICLE_CMD.VEHICLE_CMD_COMPONENT_ARM_DISARM);
    assert.equal(msg.param1, 0);
    assert.equal(msg.param2, 21196);
  });

  test('moveRelative is a DO_REPOSITION to the absolute target', () => {
    const msg = buildVehicleCommand({
      type: CommandType.MOVE_RELATIVE,
      attributes: { dx: 5, dz: 3 },
      position: POS,
    });
    assert.equal(msg.command, VEHICLE_CMD.VEHICLE_CMD_DO_REPOSITION);
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

  test('targets the system id the vehicle reports, never broadcast', () => {
    const msg = buildVehicleCommand({ type: CommandType.LAND, position: POS });
    assert.equal(msg.target_system, 3);
    assert.equal(msg.target_component, 1);
  });

  test('without a known system id the command is rejected, even one that needs no position', () => {
    const noSysId = { ...POS, attributes: {} };
    assert.throws(() => buildVehicleCommand({ type: CommandType.KILL_MOTORS, position: noSysId }), /system id/);
    assert.throws(() => buildVehicleCommand({ type: CommandType.LAND }), /system id/);
  });

  test('gimbal is a DO_GIMBAL_MANAGER_PITCHYAW', () => {
    const msg = buildVehicleCommand({ type: CommandType.GIMBAL, attributes: { pitch: -30, yaw: 10 }, position: POS });
    assert.equal(msg.command, VEHICLE_CMD.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW);
    assert.equal(msg.param1, -30);
    assert.equal(msg.param2, 10);
    assert.ok(Number.isNaN(msg.param3) && Number.isNaN(msg.param4)); // rates unset: angle only
    assert.equal(msg.param5, 16); // GIMBAL_MANAGER_FLAGS_YAW_LOCK: absolute yaw (PX4 ignores 64)
    assert.equal(msg.target_component, 0);
  });

  test('gimbal take-control makes the GCS (255/190) the primary control of every gimbal', () => {
    const msg = buildGimbalTakeControl({ position: POS });
    assert.equal(msg.command, VEHICLE_CMD.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE);
    assert.deepEqual([msg.param1, msg.param2], [255, 190]);
    assert.deepEqual([msg.param3, msg.param4], [-1, -1]); // secondary control unchanged
    assert.equal(msg.param7, 0); // all gimbal devices
    assert.deepEqual([msg.source_system, msg.source_component], [255, 190]);
    assert.equal(msg.target_system, 3);
    assert.throws(() => buildGimbalTakeControl({ position: { attributes: {} } }), /system id/);
  });

  test('a non vehicle command is rejected', () => {
    assert.throws(() => buildVehicleCommand({ type: CommandType.LOAD_MISSION }), /not a vehicle command/);
  });
});
