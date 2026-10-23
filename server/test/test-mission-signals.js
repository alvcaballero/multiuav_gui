import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import sequelize from '../common/sequelize.js';
import { positionsModel } from '../models/positions/positions.js';
import { eventsModel } from '../models/events.js';
import { signalFlightState, signalAutopilotFeedback } from '../models/mission/missionSignals.js';
import { ARM_STATE, FLIGHT_MODE, FLIGHT_PHASE } from '../config/status.js';

// These callers read the cached position WITHOUT awaiting it. While positionsModel.getByDeviceId
// was `async` they got a Promise: the anomalies, wp_reached and the event position never worked,
// on any protocol. Regression tests for that.

describe('missionSignals read the cached position', () => {
  test('signalFlightState reports disarmed, RTL and on-ground', async () => {
    const deviceId = 90001;
    await positionsModel.updatePosition({
      deviceId,
      armState: ARM_STATE.DISARMED,
      flightMode: FLIGHT_MODE.RTL,
      flightPhase: FLIGHT_PHASE.ON_GROUND,
    });
    const { anomaly } = signalFlightState(deviceId);
    assert.deepEqual(anomaly?.split(',').sort(), ['DISARMED', 'ON_GROUND', 'RTH_SUSPECTED']);
  });

  test('signalAutopilotFeedback reads the waypoint the autopilot reached', async () => {
    const deviceId = 90002;
    await positionsModel.updatePosition({ deviceId, uav_state: 'ok', mission_state: '1', wp_reached: 3 });
    assert.deepEqual(signalAutopilotFeedback(deviceId, 5), { wpEstimate: 3, confidence: 'high', anomaly: null });
  });

  test('an unknown device has no signal', () => {
    assert.deepEqual(signalFlightState(99999), { wpEstimate: null, confidence: null, anomaly: null });
  });
});

describe('events carry the device position', () => {
  test('addEvent stores where the device was', async () => {
    const device = await sequelize.models.Device.create({ name: 'evt_pos_dev', category: 'px4_mavlink', protocol: 'mavlink', attributes: { systemId: 77 } });
    await positionsModel.updatePosition({ deviceId: device.id, latitude: 43.47, longitude: -8.18, altitude: 100.2, deviceTime: new Date().toISOString() });
    await eventsModel.addEvent({ type: 'info', eventTime: new Date(), deviceId: device.id, attributes: { message: 'x' } });
    const event = await sequelize.models.Event.findOne({ where: { deviceId: device.id } });
    assert.deepEqual(event.positionId, [43.47, -8.18, 100.2]);
  });
});
