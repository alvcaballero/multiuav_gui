import sequelize from '../../common/sequelize.js';
import { chatLogger } from '../../common/logger.js';
import { Json } from 'sequelize/lib/utils';

export class ChatHistoryManager {
  /**
   * Creates a chat item and persists to database
   * @param {string} chatId - Chat identifier
   * @param {string} from - Message sender ('user', 'assistant', 'system')
   * @param {object} message - Message content object
   * @param {string} responseId - Optional OpenAI response ID for assistant messages
   * @returns {Promise<object>} The created chat item
   */
  static async addMessage(chatId, from, message, responseId = null) {
    const chatItem = {
      chatId,
      from,
      timestamp: new Date().toISOString(),
      message,
      responseId,
    };

    try {
      await this.saveMessage(chatId, chatItem);
    } catch (error) {
      chatLogger.error('Error persisting message:', error);
    }

    return chatItem;
  }

  /**
   * Get or create a chat conversation
   * @param {string} chatId - Chat identifier
   * @param {object} metadata - Optional metadata
   * @returns {Promise<object>} Chat record
   */
  static async getOrCreateChat(chatId, metadata = {}) {
    const [chat, created] = await sequelize.models.Chat.findOrCreate({
      where: { id: chatId },
      defaults: {
        id: chatId,
        name: metadata.name || null,
        metadata: metadata,
        status: 'active',
        createdAt: new Date(),
        updatedAt: new Date(),
      },
    });

    if (created) {
      chatLogger.info(`Created new chat: ${chatId}`);
    }

    return chat;
  }

  /**
   * Create a new chat with server-generated UUID
   * @param {string} name - Optional chat name
   * @returns {Promise<object>} Created chat record
   */
  static async createChat(name = null) {
    const chatId = `chat_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`;
    const now = new Date();

    const chat = await sequelize.models.Chat.create({
      id: chatId,
      name: name,
      metadata: {},
      status: 'active',
      createdAt: now,
      updatedAt: now,
    });

    chatLogger.info(`Created new chat with server ID: ${chatId}`);
    return chat;
  }

  /**
   * Get all chats with optional filters
   * @param {object} options - Filter options
   * @returns {Promise<Array>} List of chats
   */
  static async getAllChats({ status = 'active', limit = 50 } = {}) {
    const where = {};
    if (status) where.status = status;

    return await sequelize.models.Chat.findAll({
      where,
      order: [['updatedAt', 'DESC']],
      limit,
      attributes: ['id', 'name', 'status', 'createdAt', 'updatedAt'],
    });
  }

  /**
   * Update chat metadata
   * @param {string} chatId - Chat identifier
   * @param {object} updates - Fields to update
   * @returns {Promise<object|null>} Updated chat or null
   */
  static async updateChat(chatId, updates) {
    const chat = await sequelize.models.Chat.findByPk(chatId);
    if (!chat) return null;

    if (updates.name !== undefined) chat.name = updates.name;
    if (updates.status) chat.status = updates.status;
    if (updates.metadata) chat.metadata = { ...chat.metadata, ...updates.metadata };
    chat.updatedAt = new Date();

    await chat.save();
    return chat;
  }

  /**
   * Delete a chat
   * @param {string} chatId - Chat identifier
   * @param {boolean} hardDelete - If true, permanently delete; otherwise soft delete
   */
  static async deleteChat(chatId, hardDelete = false) {
    if (hardDelete) {
      await sequelize.models.ChatMessage.destroy({ where: { chatId } });
      await sequelize.models.Chat.destroy({ where: { id: chatId } });
    } else {
      await this.updateChat(chatId, { status: 'deleted' });
    }
    chatLogger.info(`Chat deleted: ${chatId}, hard: ${hardDelete}`);
  }

  /**
   * Save a single message to the database
   * @param {string} chatId - Chat identifier
   * @param {object} messageItem - Message object with from, timestamp, message
   * @returns {Promise<object>} Saved message record
   */
  static async saveMessage(chatId, messageItem) {
    try {
      await this.getOrCreateChat(chatId);

      const { from, timestamp, message } = messageItem;

      let role = message.role;
      let type = message.type || 'text';
      let content = null;

      if (message.type === 'function_call' || message.type === 'tool_call') {
        role = 'assistant';
        type = 'tool_call';
        content = `Tool call: ${message.name}`;
      } else if (message.type === 'function_call_output') {
        role = 'tool';
        type = 'tool_result';
        content = typeof message.output === 'string' ? message.output.slice(0, 500) : null;
      } else if (message.type === 'message') {
        role = message.role || 'assistant';
        type = 'text';
        if (Array.isArray(message.content)) {
          const textBlocks = message.content.filter((b) => b.type === 'output_text' || b.type === 'text');
          content = textBlocks.map((b) => b.text).join('\n');
        } else if (typeof message.content === 'string') {
          content = message.content;
        }
      } else if (message.type === 'reasoning') {
        role = 'assistant';
        type = 'reasoning';
        if (Array.isArray(message.summary)) {
          content = message.summary.map((s) => s.text || s).join('\n');
        }
      } else if (typeof message.content === 'string') {
        content = message.content;
      }

      const dbMessage = await sequelize.models.ChatMessage.create({
        chatId,
        role: role || 'unknown',
        from: from,
        type: type,
        content: content,
        messageData: message,
        status: message.status || 'completed',
        timestamp: timestamp ? new Date(timestamp) : new Date(),
        responseId: messageItem.responseId || null,
      });

      await sequelize.models.Chat.update({ updatedAt: new Date() }, { where: { id: chatId } });

      return dbMessage;
    } catch (error) {
      chatLogger.error('Error saving message:', error);
      throw error;
    }
  }

  /**
   * Load conversation history from database
   * @param {string} chatId - Chat identifier
   * @param {object} options - Pagination options
   * @returns {Promise<Array>} Array of messages in internal format
   */
  static async loadHistory(chatId, { limit = 100, offset = 0 } = {}) {
    const messages = await sequelize.models.ChatMessage.findAll({
      where: { chatId, hidden: false },
      order: [['timestamp', 'ASC']],
      limit,
      offset,
    });

    return messages.map((msg) => ({
      chatId: msg.chatId,
      from: msg.from,
      timestamp: msg.timestamp.toISOString(),
      message: msg.messageData || { role: msg.role, content: msg.content },
    }));
  }

  /**
   * Get message count for a chat
   * @param {string} chatId - Chat identifier
   * @returns {Promise<number>} Number of messages
   */
  static async getMessageCount(chatId) {
    return await sequelize.models.ChatMessage.count({ where: { chatId } });
  }

  /**
   * Returns a persistence adapter for LLM session management.
   * Decouples handlers from ChatHistoryManager's concrete implementation.
   * @returns {Object} Adapter with { getSessionId, setSessionId, clearSession }
   */
  static getSessionPersistence() {
    return {
      getSessionId: (chatId) => this.getSessionId(chatId),
      setSessionId: (chatId, sessionId) => this.setSessionId(chatId, sessionId),
      clearSession: (chatId) => this.clearSession(chatId),
    };
  }

  /**
   * Get the provider session ID for a chat (e.g., OpenAI conversation ID)
   * @param {string} chatId - Chat identifier (internal app ID)
   * @returns {Promise<string|null>} Provider session ID or null
   */
  static async getSessionId(chatId) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      return chat?.metadata?.sessionId || null;
    } catch (error) {
      chatLogger.error('Error getting session ID:', error);
      return null;
    }
  }

  /**
   * Set the provider session ID for a chat
   * @param {string} chatId - Chat identifier (internal app ID)
   * @param {string} sessionId - Provider session ID
   */
  static async setSessionId(chatId, sessionId) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      if (chat) {
        const metadata = { ...(chat.metadata || {}), sessionId, sessionCreatedAt: new Date().toISOString() };
        chat.metadata = metadata;
        chat.changed('metadata', true);
        chat.updatedAt = new Date();
        await chat.save();
        chatLogger.info(`Set session ID for chat ${chatId}: ${sessionId.substring(0, 20)}...`);
      } else {
        chatLogger.warn(`Chat ${chatId} not found when setting sessionId`);
      }
    } catch (error) {
      chatLogger.error('Error setting session ID:', error);
      throw error;
    }
  }

  /**
   * Update the chat's metadata including response ID, provider and model
   * @param {string} chatId - Chat identifier
   * @param {object} updates - Object with responseId, provider, model
   */
  static async updateChatMetadata(chatId, { responseId, provider, model }) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      if (chat) {
        const metadata = { ...(chat.metadata || {}) };

        if (responseId !== undefined) {
          metadata.lastResponseId = responseId;
          metadata.lastResponseAt = new Date().toISOString();
        }

        if (provider !== undefined) metadata.provider = provider;
        if (model !== undefined) metadata.model = model;

        chat.metadata = metadata;
        chat.changed('metadata', true);
        chat.updatedAt = new Date();
        await chat.save();

        const logRespId = responseId ? responseId.substring(0, 20) + '...' : 'null';
        chatLogger.debug(`Updated chat ${chatId} metadata: responseId=${logRespId}, provider=${provider}, model=${model}`);
      }
    } catch (error) {
      chatLogger.error('Error updating chat metadata:', error);
    }
  }

  /**
   * Clear the provider session (forces creation of new session on next message)
   * @param {string} chatId - Chat identifier
   */
  static async clearSession(chatId) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      if (chat) {
        const metadata = { ...(chat.metadata || {}), sessionId: null, lastResponseId: null };
        chat.metadata = metadata;
        chat.changed('metadata', true);
        await chat.save();
        chatLogger.info(`Session cleared for chat ${chatId}`);
      }
    } catch (error) {
      chatLogger.error('Error clearing session:', error);
    }
  }

  /**
   * Set the allowed tools for a chat (persists in metadata for consistency)
   * @param {string} chatId - Chat identifier
   * @param {Array<string>|null} allowedTools - List of allowed tool names (null = all tools allowed)
   */
  static async setAllowedTools(chatId, allowedTools) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      if (chat) {
        const metadata = { ...(chat.metadata || {}), allowedTools };
        chat.metadata = metadata;
        chat.changed('metadata', true);
        chat.updatedAt = new Date();
        await chat.save();
        chatLogger.info(`Set allowedTools for chat ${chatId}: ${allowedTools ? allowedTools.join(', ') : 'all'}`);
      } else {
        chatLogger.warn(`Chat ${chatId} not found when setting allowedTools`);
      }
    } catch (error) {
      chatLogger.error('Error setting allowedTools:', error);
      throw error;
    }
  }

  /**
   * Get the allowed tools for a chat from metadata
   * @param {string} chatId - Chat identifier
   * @returns {Promise<Array<string>|null>} List of allowed tool names or null (all allowed)
   */
  static async getAllowedTools(chatId) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      return chat?.metadata?.allowedTools;
    } catch (error) {
      chatLogger.error('Error getting allowedTools:', error);
      return undefined;
    }
  }

  /**
   * Set the agent profile for a chat (persists in metadata)
   * @param {string} chatId - Chat identifier
   * @param {string} agentProfile - Profile name ('default', 'planner', etc.)
   */
  static async setAgentProfile(chatId, agentProfile) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      if (chat) {
        const metadata = { ...(chat.metadata || {}), agentProfile };
        chat.metadata = metadata;
        chat.changed('metadata', true);
        chat.updatedAt = new Date();
        await chat.save();
        chatLogger.info(`Set agentProfile for chat ${chatId}: ${agentProfile}`);
      }
    } catch (error) {
      chatLogger.error('Error setting agentProfile:', error);
    }
  }

  /**
   * Get the agent profile for a chat from metadata
   * @param {string} chatId - Chat identifier
   * @returns {Promise<string>} Profile name or 'default'
   */
  static async getAgentProfile(chatId) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      return chat?.metadata?.agentProfile || 'default';
    } catch (error) {
      chatLogger.error('Error getting agentProfile:', error);
      return 'default';
    }
  }

  /**
   * Fork a conversation up to (and including) a specific message timestamp.
   * Creates a new chat and copies all messages up to that point.
   * @param {string} sourceChatId - Source chat identifier
   * @param {string} upToTimestamp - ISO timestamp of the last message to include
   * @param {string} name - Optional name for the new chat
   * @returns {Promise<object>} New chat record
   */
  static async forkChat(sourceChatId, upToTimestamp, name = null) {
    const newChatId = `chat_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`;
    const now = new Date();

    // Create the new chat
    const newChat = await sequelize.models.Chat.create({
      id: newChatId,
      name: name || null,
      metadata: { forkedFrom: sourceChatId, forkedAt: now.toISOString() },
      status: 'active',
      createdAt: now,
      updatedAt: now,
    });

    // Load messages from source up to and including the target timestamp
    const cutoff = new Date(upToTimestamp);
    const messages = await sequelize.models.ChatMessage.findAll({
      where: {
        chatId: sourceChatId,
      },
      order: [['timestamp', 'ASC']],
    });

    const messagesToCopy = messages.filter((m) => new Date(m.timestamp) <= cutoff);

    if (messagesToCopy.length > 0) {
      const copies = messagesToCopy.map((m) => ({
        chatId: newChatId,
        role: m.role,
        from: m.from,
        type: m.type,
        content: m.content,
        messageData: m.messageData,
        status: m.status,
        timestamp: m.timestamp,
        responseId: null, // no heredar session IDs del proveedor LLM
      }));

      await sequelize.models.ChatMessage.bulkCreate(copies);
      await sequelize.models.Chat.update({ updatedAt: now }, { where: { id: newChatId } });
    }

    chatLogger.info(`Forked chat ${sourceChatId} → ${newChatId} (${messagesToCopy.length} messages copied)`);
    return newChat;
  }

  /**
   * Check whether a chat record exists (and is active) in the database.
   * @param {string} chatId - Chat identifier
   * @returns {Promise<boolean>}
   */
  static async chatExists(chatId) {
    try {
      const chat = await sequelize.models.Chat.findOne({
        where: { id: chatId, status: 'active' },
        attributes: ['id'],
      });
      return chat !== null;
    } catch (error) {
      chatLogger.error('Error checking chat existence:', error);
      return false;
    }
  }

  /**
   * Replace the last tool_result message for a specific tool name in a chat.
   * Used to inject the real mission plan result in place of the placeholder
   * returned by the MCP tool handler.
   * @param {string} chatId - Chat identifier
   * @param {string} toolName - Name of the tool whose last result will be replaced
   * @param {object} newMessageData - New messageData object to store
   * @param {string} newContent - New summarized content (≤500 chars)
   * @returns {Promise<boolean>} true if a message was found and updated
   */
  /**
   * Hides the last tool_result for a given tool name AND its paired function_call,
   * then re-inserts both at the end of the history with the corrected output.
   * Hidden messages remain in DB for the UI but are excluded from LLM history.
   *
   * Moving both messages to the end ensures the LLM sees a coherent
   * function_call → function_call_output pair regardless of how many messages
   * were interleaved between the original call and the async result.
   *
   * @param {string} chatId      - Chat identifier
   * @param {string} toolName    - Tool name to search (e.g. 'request_mission_plan')
   * @param {string} newOutput   - New value for messageData.output (JSON string)
   * @param {string} newContent  - Summarized content (≤500 chars) for the DB content column
   * @returns {Promise<boolean>} true if the original tool_result was found and hidden
   */
  static async hideAndReplaceToolResult(chatId, toolName, newOutput, newContent) {
    // Load all tool_result rows for this chat and filter in JS to avoid
    // SQLite JSON-column LIKE quirks (key order, spacing, dialect differences).
    const candidates = await sequelize.models.ChatMessage.findAll({
      where: { chatId, type: 'tool_result' },
      order: [['timestamp', 'DESC']],
    });

    const originalToolResult = candidates.find((m) => {
      const data = m.messageData;
      if (!data) return false;
      if (typeof data === 'object') return data.name === toolName;
      if (typeof data === 'string') {
        try { return JSON.parse(data).name === toolName; } catch { return false; }
      }
      return false;
    });

    if (!originalToolResult) {
      chatLogger.warn(`[hideAndReplaceToolResult] No tool_result found for tool "${toolName}" in chat ${chatId}`);
      return false;
    }

    // Extract call_id from the placeholder tool_result to find its paired function_call
    const originalData = typeof originalToolResult.messageData === 'object'
      ? originalToolResult.messageData
      : JSON.parse(originalToolResult.messageData);
    const callId = originalData.call_id;

    // ── Hide the originalToolResult tool_result placeholder ────────────────────────────
    originalToolResult.hidden = true;
    originalToolResult.changed('hidden', true);
    await originalToolResult.save();
    chatLogger.info(`[hideAndReplaceToolResult] Hidden original tool_result for "${toolName}" in chat ${chatId} (id: ${originalToolResult.id})`);

    // ── Find and hide the paired function_call (tool_call) ───────────────────
    // It may have other messages between it and the tool_result (e.g. text
    // responses from the LLM before it dispatched the tool). We hide it so the
    // LLM does not see a dangling call without a result, then re-insert a copy
    // at the end paired with the new tool_result.
    let originalToolCall = null;
    if (callId) {
      const toolCallCandidates = await sequelize.models.ChatMessage.findAll({
        where: { chatId, type: 'tool_call' },
        order: [['timestamp', 'DESC']],
      });

      originalToolCall = toolCallCandidates.find((m) => {
        const data = typeof m.messageData === 'object' ? m.messageData : (() => {
          try { return JSON.parse(m.messageData); } catch { return null; }
        })();
        return data && (data.call_id === callId || data.id === callId);
      });

      if (originalToolCall) {
        originalToolCall.hidden = true;
        originalToolCall.changed('hidden', true);
        await originalToolCall.save();
        chatLogger.info(`[hideAndReplaceToolResult] Hidden original tool_call for "${toolName}" in chat ${chatId} (id: ${originalToolCall.id})`);
      } else {
        chatLogger.warn(`[hideAndReplaceToolResult] No tool_call found with call_id "${callId}" for tool "${toolName}" in chat ${chatId}`);
      }
    }

    const now = new Date();

    // ── Re-insert the function_call at the end (1 ms before the result) ──────
    if (originalToolCall) {
      const toolCallData = typeof originalToolCall.messageData === 'object'
        ? originalToolCall.messageData
        : JSON.parse(originalToolCall.messageData);

      await sequelize.models.ChatMessage.create({
        chatId,
        role: originalToolCall.role,
        from: originalToolCall.from,
        type: originalToolCall.type,
        content: originalToolCall.content,
        messageData: toolCallData,
        status: 'completed',
        timestamp: new Date(now.getTime() - 1),
        responseId: originalToolCall.responseId,
        hidden: false,
      });
      chatLogger.info(`[hideAndReplaceToolResult] Re-inserted tool_call for "${toolName}" at end of chat ${chatId}`);
    }

    // ── Insert the new tool_result with the real output ───────────────────────
    const newMessageData = { ...originalData, output: newOutput };

    await sequelize.models.ChatMessage.create({
      chatId,
      role: originalToolResult.role,
      from: originalToolResult.from,
      type: originalToolResult.type,
      content: typeof newContent === 'string' ? newContent.slice(0, 500) : newContent,
      messageData: newMessageData,
      status: 'completed',
      timestamp: now,
      responseId: originalToolResult.responseId,
      hidden: false,
    });

    await sequelize.models.Chat.update({ updatedAt: new Date() }, { where: { id: chatId } });
    chatLogger.info(`[hideAndReplaceToolResult] Inserted replacement tool_result for "${toolName}" in chat ${chatId}`);
    return true;
  }
}
