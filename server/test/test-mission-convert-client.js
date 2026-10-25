import { describe, test } from 'node:test';
import assert from 'node:assert/strict';
import { RuteConvert, routesToTasks } from '../../client/src/common/util/missionConvert.js';
import { ALTITUDE_REF } from '../models/mission/taskGraph.js';

// The mission editor (client) loads a plan with RuteConvert and sends it back with routesToTasks.
// A task's altitude_ref has to survive that trip: absent means relative (the editor's height over
// the takeoff point), so losing an `amsl` flies that plan its home altitude too high.
describe('client missionConvert: altitude_ref round trip', () => {
  const wp = [{ pos: [47.39797, 8.54616, 520] }];

  test('a v4 plan keeps its altitude_ref through the editor', () => {
    const task = { task_id: 'T1', device: 'vtol_px4', depends_on: [], params: {}, altitude_ref: ALTITUDE_REF.AMSL, wp };
    const plan = { version: '4', tasks: [task] };
    const [out] = routesToTasks(RuteConvert(plan));
    assert.equal(out.altitude_ref, ALTITUDE_REF.AMSL);
  });

  test('a v3 plan too', () => {
    const plan = { version: '3', route: [{ uav: 'vtol_px4', attributes: {}, altitude_ref: ALTITUDE_REF.AMSL, wp }] };
    const [task] = routesToTasks(RuteConvert(plan));
    assert.equal(task.altitude_ref, ALTITUDE_REF.AMSL);
  });

  test('an editor mission without it stays without it (relative, the default)', () => {
    const plan = { version: '3', route: [{ uav: 'vtol_px4', attributes: {}, wp }] };
    const [task] = routesToTasks(RuteConvert(plan));
    assert.ok(!('altitude_ref' in task));
  });
});
