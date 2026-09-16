import { Model, DataTypes, Sequelize } from 'sequelize';
import { Device } from './device.model.js';
import { Mission } from './mission.model.js';

const MissionRoute_TABLE = 'MissionRoute';

const MissionRouteSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  missionId: {
    type: DataTypes.INTEGER,
    references: {
      model: Mission,
      key: 'id',
    },
  },
  deviceId: {
    type: DataTypes.INTEGER,
    references: {
      model: Device,
      key: 'id',
    },
  },
  status: {
    allowNull: false,
    type: DataTypes.STRING,
    defaultValue: 'init',
  },
  currentWp: {
    type: DataTypes.INTEGER,
    defaultValue: 0,
  },
  totalWp: {
    type: DataTypes.INTEGER,
    defaultValue: 0,
  },
  initTime: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
  endTime: {
    type: DataTypes.DATE,
  },
  result: {
    type: DataTypes.JSON,
  },
  errorMessage: {
    type: DataTypes.STRING,
    allowNull: true,
  },
};

class MissionRoute extends Model {
  static associate() {}

  static config(sequelize) {
    return {
      sequelize,
      tableName: MissionRoute_TABLE,
      modelName: 'MissionRoute',
      timestamps: false,
    };
  }
}

export { MissionRoute_TABLE, MissionRouteSchema, MissionRoute };
