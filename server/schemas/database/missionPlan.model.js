import { Model, DataTypes, Sequelize } from 'sequelize';

const MissionPlan_TABLE = 'MissionPlan';

const MissionPlanSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  name: {
    type: DataTypes.STRING,
  },
  missionData: {
    allowNull: false,
    type: DataTypes.JSON,
  },
  source: {
    allowNull: false,
    type: DataTypes.STRING,
    defaultValue: 'manual',
  },
  createdAt: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
};

class MissionPlan extends Model {
  static associate() {}
  static config(sequelize) {
    return {
      sequelize,
      tableName: MissionPlan_TABLE,
      modelName: 'MissionPlan',
      timestamps: false,
    };
  }
}

export { MissionPlan_TABLE, MissionPlanSchema, MissionPlan };
