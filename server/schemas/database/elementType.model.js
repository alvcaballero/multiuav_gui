import { Model, DataTypes } from 'sequelize';

const ElementType_TABLE = 'ElementTypes';

const ElementTypeSchema = {
  id: {
    allowNull: false,
    primaryKey: true,
    type: DataTypes.STRING,
  },
  name: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  description: {
    type: DataTypes.STRING,
    defaultValue: '',
  },
  icon: {
    type: DataTypes.STRING,
    allowNull: true,
  },
  model3d: {
    type: DataTypes.STRING,
    allowNull: true,
  },
  color: {
    type: DataTypes.STRING,
    allowNull: true,
  },
  isCustom: {
    type: DataTypes.BOOLEAN,
    defaultValue: false,
  },
};

class ElementType extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: ElementType_TABLE,
      modelName: 'ElementType',
      timestamps: false,
    };
  }
}

export { ElementType_TABLE, ElementTypeSchema, ElementType };
