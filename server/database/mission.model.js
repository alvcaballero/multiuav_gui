import { Model, DataTypes, Sequelize } from 'sequelize';

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
