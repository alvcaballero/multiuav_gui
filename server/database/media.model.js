import { Model, DataTypes, Sequelize } from 'sequelize';
import { Device } from './device.model.js';
import { Route } from './routes.model.js';
const Media_TABLE = 'Media';

const MediaSchema = {
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
  RouteId: {
    type: DataTypes.INTEGER,
    references: {
      model: Route, // 'Actors' would also work
      key: 'id',
    },
  },
  deviceId: {
    type: DataTypes.INTEGER,
    references: {
      model: Device, // 'Device' would also work
      key: 'id',
    },
  },
  status: {
    allowNull: false,
    type: DataTypes.STRING,
    defaultValue: 'init', //init, doing //done //error
  },
  FileDate: {
    allowNull: false,
    type: DataTypes.DATE,
  },
  type: {
    type: DataTypes.STRING, //video, image, rosbag, lidar
  },
  path: {
    type: DataTypes.STRING, // where is save in server
  },
  attributes: {
    type: DataTypes.STRING,
  },
};

class Media extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: Media_TABLE,
      modelName: 'Media',
      timestamps: false,
    };
  }
}

export { Media_TABLE, MediaSchema, Media };
