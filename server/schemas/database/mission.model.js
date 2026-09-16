import { Model, DataTypes, Sequelize } from 'sequelize';
import { MissionPlan } from './missionPlan.model.js';

const Mission_TABLE = 'Mission';

const MissionSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  name: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  // ID assigned by the external system (ExtApp) that requested the task.
  // NOT our primary key — `id` always autoincrements internally. This is how we
  // deduplicate incoming tasks and how we address the mission back to ExtApp.
  // Null for manual missions (they have no external origin).
  externalId: {
    type: DataTypes.INTEGER,
    allowNull: true,
  },
  planId: {
    type: DataTypes.INTEGER,
    references: {
      model: MissionPlan,
      key: 'id',
    },
  },
  trigger: {
    type: DataTypes.STRING,
    defaultValue: 'automatic',
  },
  uav: {
    type: DataTypes.JSON,
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
  endTime: {
    type: DataTypes.DATE,
  },
  task: {
    type: DataTypes.JSON,
  },
  results: {
    type: DataTypes.JSON,
  },
  mission: {
    type: DataTypes.JSON,
  },
  errorMessage: {
    type: DataTypes.STRING,
    allowNull: true,
  },
};

class Mission extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: Mission_TABLE,
      modelName: 'Mission',
      timestamps: false,
    };
  }
}

export { Mission_TABLE, MissionSchema, Mission };
