import sequelize from '../../common/sequelize.js';
import { chatLogger } from '../../common/logger.js';

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
      where: { chatId },
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
}
