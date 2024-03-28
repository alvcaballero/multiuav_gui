import { Model, DataTypes, Sequelize } from 'sequelize';
import { Device } from './device.model.js';
const Event_TABLE = 'event';

const EventSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  deviceId: {
    type: DataTypes.INTEGER,
    references: {
      model: Device, // 'Actors' would also work
      key: 'id',
    },
  },
  status: {
    allowNull: false,
    type: DataTypes.STRING,
    defaultValue: 'info', //warning , info, error
  },
  createdAt: {
    allowNull: false,
    type: DataTypes.DATE,
    field: 'create_at',
    defaultValue: Sequelize.NOW,
  },
  description: {
    type: DataTypes.STRING,
  },
};

class Event extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: Event_TABLE,
      modelName: 'Event',
      timestamps: false,
    };
  }
}
export { Event_TABLE, EventSchema, Event };
