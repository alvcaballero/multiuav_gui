import { createSlice } from '@reduxjs/toolkit';

// Cap on live messages kept per conversation in Redux (browser memory). Older
// messages stay safely in the server DB and can be re-fetched via
// prependMessages when the user scrolls up (see hasMoreOlder).
const MAX_LIVE_MESSAGES = 300;

const initialState = {
  // Map of chatId -> chat object
  conversations: {},

  // Currently active chat ID
  activeChatId: null,

  // Default chat ID (created on first load)
  defaultChatId: 'default',

  // List of available chats from server (for selector)
  availableChats: [],

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
