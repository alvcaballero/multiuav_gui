import { test } from 'node:test';
import assert from 'node:assert/strict';
import { stringifyKeepingNaN } from '../models/ros/rosConnection.js';

test('NaN goes to rosbridge as the bare NaN literal, not null', () => {
  const text = stringifyKeepingNaN({ op: 'publish', msg: { param3: NaN, param4: NaN, param1: -10, name: 'NaN' } });
  assert.equal(text, '{"op":"publish","msg":{"param3":NaN,"param4":NaN,"param1":-10,"name":"NaN"}}');
});
