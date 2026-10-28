import { createSlice } from '@reduxjs/toolkit';
import { requestTitle, resolvedOutcome } from '../common/util/inputRequest';

// Cap on live messages kept per conversation in Redux (browser memory). Older
// messages stay safely in the server DB and can be re-fetched via
// prependMessages when the user scrolls up (see hasMoreOlder).
const MAX_LIVE_MESSAGES = 300;

// Resolved approvals kept per chat for the transcript. Session-only: the tool message that follows
// an approved call is what survives a reload, this is the "approved by you" line between them.
const MAX_RESOLVED_PER_CHAT = 50;

const initialState = {
  // Map of chatId -> chat object
  conversations: {},

  // Currently active chat ID
  activeChatId: null,

  // Default chat ID (created on first load)
  defaultChatId: 'default',

  // List of available chats from server (for selector)
  availableChats: [],

  // Map of chatId -> pending tool-approval requests (flight tools parked on a
  // human decision). Server-owned: this is a projection of the DB, never the
  // source of truth, so a reload re-reads it instead of losing it.
  pendingApprovals: {},

  // requestId -> why the operator's answer did NOT take effect (the server could not record or
  // deliver it). The card of that request stays, shows this and lets them try again.
  approvalFailures: {},

  // requestId -> what the operator answered ({ optionId, text }), noted when they press the button
  // so that the line shown once the request resolves can say what they did.
  approvalAnswers: {},

  // chatId -> approvals that were decided, oldest first: { requestId, title, outcome, instruction, at }.
  resolvedApprovals: {},

  // chatId -> { phase: 'running' | 'planning' | 'awaiting_input' | 'idle', label } of an open
  // turn, as the server reports it (only the eve engine does). Transient: not rebuilt on reload.
  activity: {},
  busy: {},
  stopping: {},

  // Loading states
  loading: {
    sendingMessage: false,
    loadingHistory: false,
    loadingChatList: false,
    creatingChat: false,
  },

  // Error state
  error: null,
};

const chatSlice = createSlice({
  name: 'chat',
  initialState,
  reducers: {
    // Initialize or switch to a chat conversation
    setActiveChat(state, action) {
      const chatId = action.payload;
      const previousChatId = state.activeChatId;
      state.activeChatId = chatId;

      // Create conversation if it doesn't exist
      if (!state.conversations[chatId]) {
        state.conversations[chatId] = {
          id: chatId,
          messages: [],
          hasMoreOlder: false,
          createdAt: new Date().toISOString(),
          updatedAt: new Date().toISOString(),
        };
      }

      // Migrate messages from _pending to the new chat (when server creates chat)
      if (state.conversations['_pending'] && previousChatId !== chatId) {
        const pendingMessages = state.conversations['_pending'].messages || [];
        if (pendingMessages.length > 0) {
          state.conversations[chatId].messages = [
            ...pendingMessages,
            ...state.conversations[chatId].messages,
          ];
          delete state.conversations['_pending'];
        }
      }
    },

    // Add a message to a specific chat
    addMessage(state, action) {
      const { chatId, message, from, timestamp } = action.payload;

      // Ensure conversation exists
      if (!state.conversations[chatId]) {
        state.conversations[chatId] = {
          id: chatId,
          messages: [],
          hasMoreOlder: false,
          createdAt: new Date().toISOString(),
          updatedAt: new Date().toISOString(),
        };
      }

      // Add message with timestamp
      const newMessage = {
        message: message,
        from: from,
        timestamp: timestamp || new Date().toISOString(),
        id: message.id || `msg_${Date.now()}_${Math.random()}`,
      };

      const conversation = state.conversations[chatId];
      conversation.messages.push(newMessage);
      // Keep only the most recent MAX_LIVE_MESSAGES in memory. The rest stay
      // in the server DB — hasMoreOlder tells the UI it can page them back in.
      if (conversation.messages.length > MAX_LIVE_MESSAGES) {
        conversation.messages.splice(0, conversation.messages.length - MAX_LIVE_MESSAGES);
        conversation.hasMoreOlder = true;
      }
      conversation.updatedAt = new Date().toISOString();
    },

    // Replace messages wholesale (initial history load for a chat)
    setMessages(state, action) {
      const { chatId, messages, hasMore = false } = action.payload;

      if (!state.conversations[chatId]) {
        state.conversations[chatId] = {
          id: chatId,
          messages: [],
          hasMoreOlder: false,
          createdAt: new Date().toISOString(),
          updatedAt: new Date().toISOString(),
        };
      }

      state.conversations[chatId].messages = messages;
      state.conversations[chatId].hasMoreOlder = hasMore;
      state.conversations[chatId].updatedAt = new Date().toISOString();
    },

    // Prepend an older page of messages (infinite-scroll-up pagination).
    // Dedupes by timestamp in case the boundary message is fetched twice.
    prependMessages(state, action) {
      const { chatId, messages, hasMore = false } = action.payload;
      const conversation = state.conversations[chatId];
      if (!conversation) return;

      const existingTimestamps = new Set(conversation.messages.map((m) => m.timestamp));
      const olderOnes = messages.filter((m) => !existingTimestamps.has(m.timestamp));
      conversation.messages = [...olderOnes, ...conversation.messages];
      conversation.hasMoreOlder = hasMore;
    },

    // Create a new chat conversation
    createChat(state, action) {
      const { chatId, name } = action.payload;

      state.conversations[chatId] = {
        id: chatId,
        name: name || `Chat ${Object.keys(state.conversations).length + 1}`,
        messages: [],
        hasMoreOlder: false,
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      };

      state.activeChatId = chatId;
    },

    // Delete a chat conversation
    deleteChat(state, action) {
      const chatId = action.payload;
      delete state.conversations[chatId];

      // If deleted chat was active, switch to default
      if (state.activeChatId === chatId) {
        state.activeChatId = state.defaultChatId;
      }
    },

    // Replaces the pending set for a chat — used when rehydrating from the server
    // after a reload, where the server's answer is authoritative.
    setPendingApprovals(state, action) {
      const { chatId, requests } = action.payload;
      state.pendingApprovals[chatId] = requests ?? [];
    },

    addPendingApprovals(state, action) {
      const { chatId, requests } = action.payload;
      const current = state.pendingApprovals[chatId] ?? [];
      const known = new Set(current.map((r) => r.requestId));
      state.pendingApprovals[chatId] = [
        ...current,
        ...requests.filter((r) => !known.has(r.requestId)),
      ];
    },

    noteApprovalAnswer(state, action) {
      const { requestId, optionId, text } = action.payload;
      state.approvalAnswers[requestId] = { optionId, text: text || null };
    },

    resolveApprovals(state, action) {
      const { chatId, resolutions } = action.payload;
      const pending = state.pendingApprovals[chatId] ?? [];
      const resolvedList = state.resolvedApprovals[chatId] ?? [];

      for (const { requestId, outcome } of resolutions) {
        const request = pending.find((r) => r.requestId === requestId);
        // Already recorded (the legacy card goes at once and the server confirms afterwards), or
        // not one this client was showing: nothing to add.
        if (!request || resolvedList.some((r) => r.requestId === requestId)) continue;
        const answer = state.approvalAnswers[requestId];
        resolvedList.push({
          requestId,
          title: requestTitle(request),
          outcome: resolvedOutcome(outcome, answer),
          instruction: answer?.text ?? null,
          at: new Date().toISOString(),
        });
      }
      state.resolvedApprovals[chatId] = resolvedList.slice(-MAX_RESOLVED_PER_CHAT);

      const resolved = new Set(resolutions.map((r) => r.requestId));
      state.pendingApprovals[chatId] = pending.filter((r) => !resolved.has(r.requestId));
      resolved.forEach((requestId) => {
        delete state.approvalFailures[requestId];
        delete state.approvalAnswers[requestId];
      });
    },

    setApprovalFailures(state, action) {
      // A null message clears it: the operator is trying again.
      for (const { requestId, message } of action.payload.failures) {
        if (message == null) delete state.approvalFailures[requestId];
        else state.approvalFailures[requestId] = message;
      }
    },

    setChatActivity(state, action) {
      const { chatId, phase, label } = action.payload;
      if (phase === 'idle') delete state.activity[chatId];
      else state.activity[chatId] = { phase, label: label ?? null };
    },

    setChatBusy(state, action) {
      const { chatId, busy } = action.payload;
      if (busy) state.busy[chatId] = true;
      else delete state.busy[chatId];
    },

    setChatStopping(state, action) {
      const { chatId, stopping } = action.payload;
      if (stopping) state.stopping[chatId] = true;
      else delete state.stopping[chatId];
    },

    // Update loading state
    setLoading(state, action) {
      const { key, value } = action.payload;
      state.loading[key] = value;
    },

    // Set error
    setError(state, action) {
      state.error = action.payload;
    },

    // Clear error
    clearError(state) {
      state.error = null;
    },

    // Reset all chats (for testing/debugging)
    resetChats(state) {
      state.conversations = {};
      state.activeChatId = null;
      state.error = null;
      state.busy = {};
      state.stopping = {};
      state.activity = {};
    },

    // Set available chats list from server
    setAvailableChats(state, action) {
      state.availableChats = action.payload;
    },

    // Rename a chat
    renameChat(state, action) {
      const { chatId, name } = action.payload;
      if (state.conversations[chatId]) {
        state.conversations[chatId].name = name;
      }
      // Also update in available chats list
      const chatInList = state.availableChats.find((c) => c.id === chatId);
      if (chatInList) {
        chatInList.name = name;
      }
    },
  },
});

export const chatActions = chatSlice.actions;
export const chatReducer = chatSlice.reducer;

/**
 * Fork a conversation up to (and including) the message at messageTimestamp.
 * Calls the backend, creates the new chat in Redux, and switches to it.
 * @param {string} sourceChatId
 * @param {string} messageTimestamp - ISO timestamp of the last message to include
 * @returns {Function} Redux thunk
 */
export const forkConversation = (sourceChatId, messageTimestamp) => async (dispatch) => {
  try {
    const response = await fetch(`/api/chat/chats/${sourceChatId}/fork`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ messageTimestamp }),
    });

    if (!response.ok) {
      const err = await response.json();
      throw new Error(err.error || 'Fork failed');
    }

    const newChat = await response.json();

    dispatch(chatActions.setActiveChat(newChat.id));

    // Load the forked history into Redux
    const historyResponse = await fetch(`/api/chat/history/${newChat.id}`);
    if (historyResponse.ok) {
      const data = await historyResponse.json();
      if (data.messages?.length > 0) {
        dispatch(
          chatActions.setMessages({
            chatId: newChat.id,
            messages: data.messages,
            hasMore: data.hasMore ?? false,
          }),
        );
      }
    }

    return newChat;
  } catch (error) {
    dispatch(chatActions.setError(error.message));
    throw error;
  }
};
