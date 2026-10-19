import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { CommandType, COMMAND_CATALOG } from '../config/commandCatalog.js';
import { CommandType as ClientCommandType } from '../../client/src/common/util/commandTypes.js';

// The client keeps its own copy of the command types (it can't import the server).
// A rename on one side only would make the UI send a type the server rejects.
describe('client CommandType mirror', () => {
  test('matches the server CommandType key by key', () => {
    assert.deepEqual({ ...ClientCommandType }, { ...CommandType });
  });

  test('every command type has a catalog entry', () => {
    for (const type of Object.values(CommandType)) assert.ok(COMMAND_CATALOG[type], type);
  });
});
