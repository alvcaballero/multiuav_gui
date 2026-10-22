import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { TOOL_APPROVAL_REQUIRED } from '../config/config.js';

// The eve repo is a sibling checkout (px4/gcs_eevee_assistant), not a dependency: when it is not
// there the test says so instead of passing silently.
const EVE_CONNECTION = join(
  dirname(fileURLToPath(import.meta.url)),
  '../../../../gcs_eevee_assistant/agents/assistant/agent/connections/multiuav-gcs.ts'
);

/**
 * Two lists decide which tools need a person: this server's `TOOL_APPROVAL_REQUIRED` (legacy chats)
 * and eve's `FLIGHT_CRITICAL_TOOLS` (eve chats). Nothing makes them one list, so a tool added to
 * one and not the other would fly with no gate on one of the two engines.
 *
 * The relation that must hold is containment, not equality: eve may gate MORE (it also lists
 * `send_command`, which its assistant cannot reach) but never fewer.
 */
describe('flight tools: this server and eve must agree on what needs approval', () => {
  const present = existsSync(EVE_CONNECTION);

  it(
    'every tool this server gates is also gated by eve',
    { skip: !present && `eve repo not found at ${EVE_CONNECTION}` },
    () => {
      const source = readFileSync(EVE_CONNECTION, 'utf8');
      const list = source.match(/FLIGHT_CRITICAL_TOOLS\s*=\s*\[([^\]]*)\]/);
      assert.ok(
        list,
        'could not find FLIGHT_CRITICAL_TOOLS in the eve connection: update this test with its new shape'
      );

      const eveTools = [...list[1].matchAll(/["']([^"']+)["']/g)].map((match) => match[1]);
      assert.ok(eveTools.length > 0);

      // eve matches by suffix (`endsWith`) because MCP tool names arrive prefixed with `multiuav-gcs__`.
      const ungated = TOOL_APPROVAL_REQUIRED.filter(
        (tool) => !eveTools.some((eveTool) => `multiuav-gcs__${tool}`.endsWith(eveTool))
      );
      assert.deepEqual(ungated, [], `gated here but NOT in eve: ${ungated.join(', ')}`);
    }
  );
});
