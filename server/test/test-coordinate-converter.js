import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { convertMissionXYZToLatLong } from '../models/mission/coordinateConverter.js';

const origin = { lat: 37.4, lng: -6.0, alt: 0 };

describe('convertMissionXYZToLatLong', () => {
  test('converts the wp of a tasks[] mission and keeps every other field', () => {
    const task = {
      task_id: 'T2',
      device: 'uav_1',
      action: 'INSPECT',
      depends_on: ['T1'],
      params: { max_vel: 5, mode_gimbal: 1 },
      uav_type: 'px4',
      wp: [{ type: 'inspection', pos: [0, 0, 30], yaw: 90, gimbal: -45, speed: 3, target_id: 7, zone: 'tip' }],
    };
    const out = convertMissionXYZToLatLong({ version: '4', name: 'm', global_origin: origin, tasks: [task] });

    assert.equal(out.global_origin, undefined);
    const [converted] = out.tasks;
    assert.deepEqual(converted.wp[0].pos, [origin.lat, origin.lng, 30]);
    const { wp: _wp, ...taskFields } = converted;
    const { wp: _src, ...srcFields } = task;
    assert.deepEqual(taskFields, srcFields);
    const { pos: _pos, ...wpFields } = converted.wp[0];
    const { pos: _srcPos, ...srcWpFields } = task.wp[0];
    assert.deepEqual(wpFields, srcWpFields);
  });

  test('still converts legacy route[]', () => {
    const out = convertMissionXYZToLatLong({ global_origin: origin, route: [{ uav: 'uav_1', wp: [{ pos: [0, 0, 10] }] }] });
    assert.deepEqual(out.route[0].wp[0].pos, [origin.lat, origin.lng, 10]);
  });
});
