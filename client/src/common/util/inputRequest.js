/**
 * Helpers for the card that asks the operator to decide (`ToolApprovalGate`).
 *
 * Two engines produce the requests it shows and they do not speak the same dialect: the legacy one
 * parks a tool call and times it out (`expiresAt`, `inputHash`, options `approve` / `deny`), eve
 * sends its own `options` (`approve` / `cancel`) and has neither expiry nor hash. The card renders
 * the request it is given and does not know which engine it came from; what differs is handled here.
 */

const LEGACY_OPTIONS = [
  { id: 'approve', label: 'Approve & execute' },
  { id: 'deny', label: 'Deny' },
];

/** What the operator can answer with. A request from before `options` existed gets the legacy pair. */
export const optionsOf = (request) => (request.options?.length ? request.options : LEGACY_OPTIONS);

/**
 * The first option is the affirmative one in every request the servers send (approve / continue);
 * the rest are ways to say no. The card makes the first one the button that acts at once and gives
 * the others a confirmation step with an optional reason.
 */
export const splitOptions = (request) => {
  const [affirmative, ...negatives] = optionsOf(request);
  return { affirmative, negatives };
};

/** eve names MCP tools `<server>__<tool>`; the operator reads the tool. */
export const displayToolName = (name) => (name ? name.split('__').pop() : null);

export const requestTitle = (request) =>
  displayToolName(request.toolName ?? request.action?.toolName) ??
  request.prompt ??
  'Waiting for your input';

/** Only requests that expire carry `expiresAt` (legacy). eve's wait for as long as the session lives. */
export const isTimed = (request) => Boolean(request.expiresAt);

export const secondsLeft = (expiresAt, now = Date.now()) =>
  Math.max(0, Math.floor((new Date(expiresAt).getTime() - now) / 1000));

export const formatCountdown = (seconds) =>
  `${Math.floor(seconds / 60)}:${String(seconds % 60).padStart(2, '0')}`;

/** A request that arms an aircraft is shown in red; a session limit or a question is not an alarm. */
export const isFlightApproval = (request) =>
  (request.kind ?? 'tool-approval') === 'tool-approval' && Boolean(request.action);

/** One line for the status under the messages while a turn is open. */
export const activityText = (activity) => {
  if (!activity) return null;
  if (activity.phase === 'planning')
    return `Working on ${displayToolName(activity.label) ?? 'a task'}…`;
  if (activity.phase === 'running') return 'Thinking…';
  return null;
};
