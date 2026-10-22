import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const FIXTURES = join(dirname(fileURLToPath(import.meta.url)), '..', 'fixtures', 'eve');

/**
 * An eve stream captured as NDJSON, parsed.
 *
 * `subagent` and `child-session` are real captures (eve 0.81.1, stream v26) with the text deltas
 * trimmed to the first three. The `*.synthetic` ones were built by hand from the shapes eve's
 * authors reported, because the capture needs the GCS MCP server up: swap them for real captures
 * when they exist (the tests only rely on the shapes, not on the ids).
 */
export const loadEveFixture = (name) =>
  readFileSync(join(FIXTURES, `${name}.ndjson`), 'utf8')
    .split('\n')
    .filter(Boolean)
    .map((line) => JSON.parse(line));

/** A bare event, for the cases a fixture does not cover. */
let counter = 0;
export const eveEvent = (type, data = {}) => ({
  type,
  data,
  meta: { id: `evt_T${++counter}`, at: new Date(0).toISOString() },
});
