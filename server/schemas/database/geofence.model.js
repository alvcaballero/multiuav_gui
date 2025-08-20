import { area } from '@turf/turf';
import { Model, DataTypes, Sequelize } from 'sequelize';

const Geofence_TABLE = 'Geofence';

const GeofenceSchema = {
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
  description: {
    type: DataTypes.STRING,
    defaultValue: '',
  },
  area: {
    type: DataTypes.STRING,
  },
  attributes: {
    type: DataTypes.JSON,
    defaultValue: {},
  },
};

class Geofence extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: Geofence_TABLE,
      modelName: 'Geofence',
      timestamps: false,
    };
  }
}

export { Geofence_TABLE, GeofenceSchema, Geofence };
