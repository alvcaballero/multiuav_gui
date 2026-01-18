import { Model, DataTypes, Sequelize } from 'sequelize';

const Chat_TABLE = 'Chat';

const ChatSchema = {
  id: {
    allowNull: false,
    primaryKey: true,
    type: DataTypes.STRING,
  },
  name: {
    type: DataTypes.STRING,
    allowNull: true,
  },
  status: {
    type: DataTypes.STRING,
    defaultValue: 'active',
  },
  metadata: {
    type: DataTypes.JSON,
    defaultValue: {},
  },
  createdAt: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
  updatedAt: {
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
};

class Chat extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: Chat_TABLE,
      modelName: 'Chat',
      timestamps: false,
    };
  }
}

export { Chat_TABLE, ChatSchema, Chat };
