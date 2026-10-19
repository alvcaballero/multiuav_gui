/**
 * Answers a parked tool approval.
 *
 * Deliberately NOT a chat message: typing "yes, go ahead" must never authorise a
 * flight tool. Only this structured response, keyed by `requestId`, decides —
 * with several requests open there is no way to tell which one free text meant.
 *
 * @param {string} chatId
 * @param {Array<{requestId: string, optionId: 'approve'|'deny', text?: string}>} responses
 * @param {string|null} responderPrincipalId
 */
export const sendToolApprovalResponse = (chatId, responses, responderPrincipalId = null) => {
  const socket = window.websocket;

  if (!socket || socket.readyState !== WebSocket.OPEN) {
    throw new Error('WebSocket not connected');
  }

  socket.send(
    JSON.stringify({
      type: 'chat:tool_approval_response',
      payload: { chatId, responses, responderPrincipalId },
    }),
  );
};

/**
 * Re-reads the still-pending approvals for a chat.
 *
 * The server is the only source of truth here: a parked approval lives in the DB,
 * so a reload asks for it again rather than relying on anything kept in browser.
 */
export const fetchPendingApprovals = async (chatId) => {
  const response = await fetch(`/api/chat/approvals/${chatId}`);
  if (!response.ok) throw new Error(`Failed to load pending approvals (${response.status})`);
  const { requests } = await response.json();
  return requests ?? [];
};
