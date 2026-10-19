import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { encodeRosSrv } from '../models/ros/rosEncode.js';

// El objeto de dominio del gimbal son ángulos ABSOLUTOS en grados (commandsModel.GimbalUAV).
// Cada dialecto tiene que pedirle a su firmware el modo absoluto, no el incremental.
const GIMBAL = { reset: false, pitch: -30, roll: 0, yaw: 45 };

describe('encodeRosSrv — Gimbal', () => {
  test('OSDK pide rotación absoluta y deja los grados tal cual', () => {
    const out = encodeRosSrv({ type: 'Gimbal', msg: GIMBAL, msgType: 'dji_osdk_ros/GimbalAction' });
    assert.equal(out.rotationMode, 1); // 0 sería incremental sobre el ángulo actual
    assert.equal(out.pitch, -30);
    assert.equal(out.yaw, 45);
    assert.equal(out.is_reset, false);
  });

  test('PSDK pide rotación absoluta y convierte a radianes', () => {
    const out = encodeRosSrv({ type: 'Gimbal', msg: GIMBAL, msgType: 'psdk_interfaces/msg/GimbalRotation' });
    assert.equal(out.rotation_mode, 1);
    assert.ok(Math.abs(out.pitch - (-30 * Math.PI) / 180) < 1e-9);
    assert.ok(Math.abs(out.yaw - (45 * Math.PI) / 180) < 1e-9);
  });
});
