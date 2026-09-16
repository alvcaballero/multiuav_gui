import { Model, DataTypes } from 'sequelize';

const Assignment_TABLE = 'Assignments';

const AssignmentSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  baseId: {
    allowNull: false,
    type: DataTypes.INTEGER,
  },
  deviceId: {
    allowNull: false,
    type: DataTypes.INTEGER,
  },
  settings: {
    type: DataTypes.JSON,
    defaultValue: {},
  },
};

class Assignment extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: Assignment_TABLE,
      modelName: 'Assignment',
      timestamps: false,
    };
  }
}

export { Assignment_TABLE, AssignmentSchema, Assignment };
