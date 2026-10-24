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
  protocol: {
    type: DataTypes.STRING,
  },
  // What the device is (config/devices/device_models/<key>.yaml); `category` is how it is talked to.
  deviceModel: {
    type: DataTypes.STRING,
    allowNull: true,
  },
  // Settings specific to each device that depend on something else about it, e.g. its protocol
  // (mavlink: { systemId }). Which keys apply when lives in schemas/zod/devices.js (ATTRIBUTE_GROUPS).
  attributes: {
    type: DataTypes.JSON,
    allowNull: true,
  },
  camera: {
    type: DataTypes.JSON,
  },
  files: {
    type: DataTypes.JSON,
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
  deletedAt: {
    type: DataTypes.DATE,
    allowNull: true,
    defaultValue: null,
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
