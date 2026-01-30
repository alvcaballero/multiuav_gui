import sequelize from '../../common/sequelize.js';
import { chatLogger } from '../../common/logger.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';

export class ChatPersistenceModel {
  /**
   * Creates a chat item, emits event and persists to database
   * @param {string} chatId - Chat identifier
   * @param {string} from - Message sender ('user', 'assistant', 'system')
   * @param {object} message - Message content object
   * @param {Array} conversationHistory - Reference to conversation history array to push to
   * @param {string} responseId - Optional OpenAI response ID for assistant messages
   * @returns {Promise<object>} The created chat item
   */
  static async addMessageToChat(chatId, from, message, conversationHistory, responseId = null) {
    const chatItem = {
      chatId,
      from,
      timestamp: new Date().toISOString(),
      message,
      responseId,
    };

    if (from === 'assistant') {
      eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, chatItem);
    }

    conversationHistory.push(chatItem);

    try {
      await this.saveMessage(chatId, chatItem);
    } catch (error) {
      chatLogger.error('Error persisting message:', error);
      // Don't throw - continue processing even if DB fails
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

      // Determine role based on message type
      // Tool calls from LLM: { type: 'function_call', name: '...', call_id: '...', arguments: '...' }
      // Tool results: { type: 'function_call_output', call_id: '...', output: '...' }
      // OpenAI Responses API message: { type: 'message', role: '...', content: [...] }
      // OpenAI reasoning: { type: 'reasoning', summary: [...] }
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
        // OpenAI Responses API format: { type: 'message', role: '...', content: [{type: 'output_text', text: '...'}] }
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
   * Get the last response ID for a chat (DEPRECATED - use getConversationId)
   * @param {string} chatId - Chat identifier
   * @returns {Promise<string|null>} Last response ID or null
   */
  static async getLastResponseId(chatId) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      return chat?.metadata?.lastResponseId || null;
    } catch (error) {
      chatLogger.error('Error getting last response ID:', error);
      return null;
    }
  }

  /**
   * Get the OpenAI conversation ID for a chat
   * @param {string} chatId - Chat identifier (internal app ID)
   * @returns {Promise<string|null>} OpenAI conversation ID or null
   */
  static async getConversationId(chatId) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      return chat?.metadata?.conversationId || null;
    } catch (error) {
      chatLogger.error('Error getting conversation ID:', error);
      return null;
    }
  }

  /**
   * Set the OpenAI conversation ID for a chat
   * @param {string} chatId - Chat identifier (internal app ID)
   * @param {string} conversationId - OpenAI conversation ID
   */
  static async setConversationId(chatId, conversationId) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      if (chat) {
        const metadata = { ...(chat.metadata || {}), conversationId, conversationCreatedAt: new Date().toISOString() };
        chat.metadata = metadata;
        chat.changed('metadata', true); // Force Sequelize to detect JSON change
        chat.updatedAt = new Date();
        await chat.save();
        chatLogger.info(`Set conversation ID for chat ${chatId}: ${conversationId.substring(0, 20)}...`);
      } else {
        chatLogger.warn(`Chat ${chatId} not found when setting conversationId`);
      }
    } catch (error) {
      chatLogger.error('Error setting conversation ID:', error);
      throw error;
    }
  }

  /**
   * Update the chat's metadata including conversation ID, response ID, provider and model
   * @param {string} chatId - Chat identifier
   * @param {object} updates - Object with conversationId, responseId, provider, model
   */
  static async updateChatMetadata(chatId, { conversationId, responseId, provider, model }) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      if (chat) {
        const metadata = { ...(chat.metadata || {}) };

        // NEW: Conversation ID (OpenAI Conversations API)
        if (conversationId !== undefined) {
          metadata.conversationId = conversationId;
          if (conversationId) {
            metadata.conversationCreatedAt = metadata.conversationCreatedAt || new Date().toISOString();
          }
        }

        // DEPRECATED: Response ID (legacy response chaining)
        if (responseId !== undefined) {
          metadata.lastResponseId = responseId;
          metadata.lastResponseAt = new Date().toISOString();
        }

        if (provider !== undefined) metadata.provider = provider;
        if (model !== undefined) metadata.model = model;

        chat.metadata = metadata;
        chat.changed('metadata', true); // Force Sequelize to detect JSON change
        chat.updatedAt = new Date();
        await chat.save();

        const logConvId = conversationId ? conversationId.substring(0, 20) + '...' : 'null';
        const logRespId = responseId ? responseId.substring(0, 20) + '...' : 'null';
        chatLogger.debug(`Updated chat ${chatId} metadata: conversationId=${logConvId}, responseId=${logRespId}, provider=${provider}, model=${model}`);
      }
    } catch (error) {
      chatLogger.error('Error updating chat metadata:', error);
    }
  }

  /**
   * Clear the conversation (forces creation of new conversation on next message)
   * Also clears response chain for backwards compatibility
   * @param {string} chatId - Chat identifier
   */
  static async clearConversation(chatId) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      if (chat) {
        const metadata = { ...(chat.metadata || {}), conversationId: null, lastResponseId: null };
        chat.metadata = metadata;
        chat.changed('metadata', true); // Force Sequelize to detect JSON change
        await chat.save();
        chatLogger.info(`Conversation cleared for chat ${chatId}`);
      }
    } catch (error) {
      chatLogger.error('Error clearing conversation:', error);
    }
  }

  /**
   * Clear the response chain (forces full history on next message)
   * DEPRECATED - use clearConversation instead
   * @param {string} chatId - Chat identifier
   */
  static async clearResponseChain(chatId) {
    try {
      const chat = await sequelize.models.Chat.findByPk(chatId);
      if (chat) {
        const metadata = chat.metadata || {};
        metadata.lastResponseId = null;
        chat.metadata = metadata;
        await chat.save();
        chatLogger.info(`Response chain cleared for chat ${chatId}`);
      }
    } catch (error) {
      chatLogger.error('Error clearing response chain:', error);
    }
  }
}
