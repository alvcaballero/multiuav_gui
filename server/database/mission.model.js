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
    unique: true,
  },
  status: {
    allowNull: false,
    type: DataTypes.STRING,
    defaultValue: 'init',
  },
  mission: {
    type: DataTypes.STRING,
  },
  initTime: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
  FinishTime: {
    type: DataTypes.DATE,
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
