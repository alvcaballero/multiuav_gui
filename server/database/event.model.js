import { Model, DataTypes, Sequelize } from 'sequelize';
import { Device } from './device.model.js';
import { Mission } from './mission.model.js';

const Event_TABLE = 'event';

const EventSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  type: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  eventTime: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
  deviceId: {
    type: DataTypes.INTEGER,
    references: {
      model: Device, // 'Actors' would also work
      key: 'id',
    },
  },
  positionId: {
    type: DataTypes.INTEGER,
  },
  missionId: {
    type: DataTypes.INTEGER,
    references: {
      model: Mission, // 'Actors' would also work
      key: 'id',
    },
  },
  attributes: {
    type: DataTypes.JSON,
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
