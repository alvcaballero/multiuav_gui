import { Model, DataTypes, Sequelize } from 'sequelize';
import { Device } from './device.model.js';

const PositionHistory_TABLE = 'PositionHistory';

const PositionHistorySchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  deviceId: {
    allowNull: false,
    type: DataTypes.INTEGER,
    references: {
      model: Device,
      key: 'id',
    },
  },
  // Reloj del server al muestrear: coherente para ordenar/consultar.
  fixTime: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
  // Timestamp reportado por el dispositivo (puede faltar o ser inconsistente
  // según el payload; se guarda tal cual para reconciliar después).
  deviceTime: {
    allowNull: true,
    type: DataTypes.DATE,
  },
  latitude: {
    type: DataTypes.FLOAT,
  },
  longitude: {
    type: DataTypes.FLOAT,
  },
  altitude: {
    type: DataTypes.FLOAT,
  },
  course: {
    type: DataTypes.FLOAT,
  },
  speed: {
    type: DataTypes.FLOAT,
  },
  // Snapshot de attributes: batería, gimbal, obstacle_info, estados y sensores
  // (MIC/Metano/CO...). Se guarda tal cual llega desde positionsModel.
  attributes: {
    type: DataTypes.JSON,
  },
};

class PositionHistory extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: PositionHistory_TABLE,
      modelName: 'PositionHistory',
      timestamps: false,
      // Índice compuesto para consultas de traza por dispositivo + rango de tiempo.
      indexes: [{ name: 'position_history_device_time', fields: ['deviceId', 'fixTime'] }],
    };
  }
}

export { PositionHistory_TABLE, PositionHistorySchema, PositionHistory };
