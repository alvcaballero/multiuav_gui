import { createSlice } from '@reduxjs/toolkit';

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

      state.conversations[chatId].messages.push(newMessage);
      state.conversations[chatId].updatedAt = new Date().toISOString();
    },

    // Add multiple messages (for loading history)
    setMessages(state, action) {
      const { chatId, messages } = action.payload;

      if (!state.conversations[chatId]) {
        state.conversations[chatId] = {
          id: chatId,
          messages: [],
          createdAt: new Date().toISOString(),
          updatedAt: new Date().toISOString(),
        };
      }

      state.conversations[chatId].messages = messages;
      state.conversations[chatId].updatedAt = new Date().toISOString();
    },

    // Create a new chat conversation
    createChat(state, action) {
      const { chatId, name } = action.payload;

      state.conversations[chatId] = {
        id: chatId,
        name: name || `Chat ${Object.keys(state.conversations).length + 1}`,
        messages: [],
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
