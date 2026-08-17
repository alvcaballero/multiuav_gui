import { Model, DataTypes } from 'sequelize';

const Base_TABLE = 'Bases';

const BaseSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  typeId: {
    type: DataTypes.STRING,
    allowNull: true,
  },
  name: {
    type: DataTypes.STRING,
    allowNull: true,
  },
  latitude: {
    type: DataTypes.FLOAT,
  },
  longitude: {
    type: DataTypes.FLOAT,
  },
  corners: {
    type: DataTypes.JSON,
    allowNull: true,
  },
};

class Base extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: Base_TABLE,
      modelName: 'Base',
      timestamps: false,
    };
  }
}

export { Base_TABLE, BaseSchema, Base };
