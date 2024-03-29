import { Model, DataTypes, Sequelize } from 'sequelize';
import { Device } from './device.model.js';
import { Mission } from './mission.model.js';
const Route_TABLE = 'Route';

const RouteSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  missionId: {
    type: DataTypes.INTEGER,
    references: {
      model: Mission, // 'Actors' would also work
      key: 'id',
    },
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
    defaultValue: 'init',
  },

  initTime: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
  FinishTime: {
    type: DataTypes.DATE,
  },
  result: {
    type: DataTypes.STRING,
  },
};

class Route extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: Route_TABLE,
      modelName: 'Route',
      timestamps: false,
    };
  }
}

export { Route_TABLE, RouteSchema, Route };
