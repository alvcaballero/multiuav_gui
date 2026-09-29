import { Model, DataTypes, Sequelize } from 'sequelize';
import { Device } from './device.model.js';
import { Mission } from './mission.model.js';
import { LEGACY_ROUTE_ACTION } from '../../models/mission/taskGraph.js';

const MissionTask_TABLE = 'MissionTask';

const MissionTaskSchema = {
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
  // The plan's task_id ("T1"...): unique within a mission, referenced by dependsOn.
  taskKey: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  dependsOn: {
    allowNull: false,
    type: DataTypes.JSON,
    defaultValue: [],
  },
  action: {
    allowNull: false,
    type: DataTypes.STRING,
    defaultValue: LEGACY_ROUTE_ACTION,
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

class MissionTask extends Model {
  static associate() {}

  static config(sequelize) {
    return {
      sequelize,
      tableName: MissionTask_TABLE,
      modelName: 'MissionTask',
      timestamps: false,
      indexes: [{ name: 'mission_task_mission_id_task_key', unique: true, fields: ['missionId', 'taskKey'] }],
    };
  }
}

export { MissionTask_TABLE, MissionTaskSchema, MissionTask };
