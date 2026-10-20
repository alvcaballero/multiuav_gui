import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { encodeMsg, hasMsgEncoder } from '../models/msgEncode.js';

// Golden: what the old rosEncode.encodeRosSrv returned for each msgType (generated before
// the move). Same input → same message, so moving the encoding changed nothing on the wire.
const golden = JSON.parse(readFileSync(new URL('./api/json/msg_encode_golden.json', import.meta.url)));

describe('encodeMsg — same output as the old rosEncode', () => {
  for (const { msgType, input, output } of golden) {
    test(msgType, () => {
      assert.deepEqual(encodeMsg(msgType, input), output);
    });
  }
});

describe('encodeMsg', () => {
  test('no message → empty request', () => {
    assert.deepEqual(encodeMsg('std_srvs/SetBool', undefined), {});
  });

  test('an unregistered msgType goes out unchanged (raw routes), and says it has no encoder', () => {
    const msg = { a: 1 };
    assert.equal(encodeMsg('some_pkg/msg/Unknown', msg), msg);
    assert.equal(hasMsgEncoder('some_pkg/msg/Unknown'), false);
    assert.equal(hasMsgEncoder('dji_osdk_ros/GimbalAction'), true);
  });

  test('a download date that is not UTC ISO is rejected', () => {
    const msg = { startDate: '2026-07-13 14:30', endDate: '2026-07-13T15:30:00.000Z' };
    assert.throws(() => encodeMsg('dji_osdk_ros/DownloadMedia', msg), /UTC ISO 8601/);
  });
});
