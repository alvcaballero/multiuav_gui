import { Model, DataTypes, Sequelize } from 'sequelize';

const Device_TABLE = 'Devices';

const DeviceSchema = {
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
  category: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  model: {
    type: DataTypes.STRING,
  },
  ip: {
    type: DataTypes.STRING,
  },
  user: {
    type: DataTypes.STRING,
  },
  pwd: {
    type: DataTypes.STRING,
  },
  camera: {
    type: DataTypes.STRING,
  },
  status: {
    type: DataTypes.STRING,
  },
  lastUpdate: {
    type: DataTypes.DATE,
  },
  createAt: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
};

class Device extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: Device_TABLE,
      modelName: 'Device',
      timestamps: false,
    };
  }
}

export { Device_TABLE, DeviceSchema, Device };
