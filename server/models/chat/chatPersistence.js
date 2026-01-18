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
   * @returns {Promise<object>} The created chat item
   */
  static async addMessageToChat(chatId, from, message, conversationHistory) {
    const chatItem = {
      chatId,
      from,
      timestamp: new Date().toISOString(),
      message,
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

      const dbMessage = await sequelize.models.ChatMessage.create({
        chatId,
        role: message.role,
        from: from,
        type: message.type || 'text',
        content: typeof message.content === 'string' ? message.content : null,
        messageData: message,
        status: message.status || 'completed',
        timestamp: timestamp ? new Date(timestamp) : new Date(),
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
}
