import { Model, DataTypes } from 'sequelize';

const ElementGroup_TABLE = 'ElementGroups';

const ElementGroupSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  typeId: {
    allowNull: false,
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
  linea: {
    type: DataTypes.BOOLEAN,
    defaultValue: false,
  },
  attributes: {
    type: DataTypes.JSON,
    defaultValue: {},
  },
  deletedAt: {
    type: DataTypes.DATE,
    allowNull: true,
    defaultValue: null,
  },
};

class ElementGroup extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: ElementGroup_TABLE,
      modelName: 'ElementGroup',
      timestamps: false,
    };
  }
}

export { ElementGroup_TABLE, ElementGroupSchema, ElementGroup };
