import { Model, DataTypes, Sequelize } from 'sequelize';
import { Device } from './device.model.js';
import { Route } from './routes.model.js';
import { Mission } from './mission.model.js';
const File_TABLE = 'File';

const FileSchema = {
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
  routeId: {
    type: DataTypes.INTEGER,
    references: {
      model: Route, // 'Actors' would also work
      key: 'id',
    },
  },
  missionId: {
    type: DataTypes.INTEGER,
    references: {
      model: Mission, // 'Actors' would also work
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
    type: DataTypes.INTEGER,
    defaultValue: 0, //init, doing //done //error
  },
  type: {
    type: DataTypes.STRING, //video, image, rosbag, lidar
  },
  path: {
    type: DataTypes.STRING, // where is save in server
  },
  source: {
    type: DataTypes.STRING, // where is save in server
  },
  path2: {
    type: DataTypes.STRING, // where is save in server
  },
  date: {
    allowNull: false,
    type: DataTypes.DATE,
  },
  attributes: {
    type: DataTypes.STRING,
  },
};

class File extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: File_TABLE,
      modelName: 'File',
      timestamps: false,
    };
  }
}

export { File_TABLE, FileSchema, File };
