import { Model, DataTypes } from 'sequelize';

const ElementItem_TABLE = 'ElementItems';

const ElementItemSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  groupId: {
    allowNull: false,
    type: DataTypes.INTEGER,
  },
  name: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  latitude: {
    type: DataTypes.FLOAT,
  },
  longitude: {
    type: DataTypes.FLOAT,
  },
  description: {
    type: DataTypes.STRING,
    allowNull: true,
  },
  attributes: {
    type: DataTypes.JSON,
    allowNull: true,
  },
};

class ElementItem extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: ElementItem_TABLE,
      modelName: 'ElementItem',
      timestamps: false,
    };
  }
}

export { ElementItem_TABLE, ElementItemSchema, ElementItem };
