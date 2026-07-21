import { eventsController } from '../../controllers/events.js';
import { round } from '../../common/utils.js';
import { logger } from '../../common/logger.js';
const positions = {};
const camera = {};

export class positionsModel {
  static async getAll(query) {
    return positions;
  }

  static async getByDeviceId(deviceId) {
    return positions[deviceId];
  }
  // Posiciones cacheadas de una lista de ids (para GET /positions?id=31&id=42).
  // Devuelve solo las que existen en caché, como array.
  static async getByDeviceIds(ids) {
    return ids.map((id) => positions[id]).filter((p) => p !== undefined);
  }
  static async getCamera() {
    return camera;
  }
  /**
   * Precarga la caché en RAM con filas del histórico (una por device), para que
   * el mapa no aparezca vacío al arrancar hasta que llegue telemetría nueva.
   * No pisa un device que ya tenga posición viva en caché.
   * @param {object[]} rows - filas planas de PositionHistory.
   * @returns {number} cantidad de devices hidratados.
   */
  static hydrate(rows) {
    let count = 0;
    for (const row of rows) {
      if (!row || row.deviceId === undefined || row.deviceId === null) continue;
      if (positions[row.deviceId] !== undefined) continue; // ya hay dato vivo
      positions[row.deviceId] = {
        deviceId: row.deviceId,
        latitude: row.latitude ?? undefined,
        longitude: row.longitude ?? undefined,
        altitude: row.altitude ?? undefined,
        course: row.course ?? 0.0,
        speed: row.speed ?? 0.0,
        accuracy: 0.0,
        deviceTime: row.deviceTime ?? row.fixTime ?? undefined,
        attributes: { ...(row.attributes || {}) },
      };
      count += 1;
    }
    logger.info(`positionsModel hydrated ${count} device(s) from history`);
    return count;
  }
  static updateCamera(payload) {
    camera[payload.deviceId] = payload;
  }
  static removePosition({ id }) {
    delete positions[id];
    logger.debug(`Position removed for device id=${id}`);
  }
  static async updatePosition(payload) {
    if (payload === null) {
      return null;
    }

    if (positions[payload.deviceId] === undefined) {
      positions[payload.deviceId] = {
        deviceId: payload.deviceId,
        accuracy: 0.0,
        speed: 0.0,
        course: 0.0,
        attributes: {
          batteryLevel: 0,
          gimbal: [0, 0, 0],
          obstacle_info: [100, 100, 100, 100, 100, 100],
          takeoff_height: 400,
          mission_state: 'Ready',
          wp_reached: null,
          uav_state: 'OK',
          landed_state: 'Ready',
          alarm: 'UNDEFINED',
        },
      };
    }
    if (payload.hasOwnProperty('latitude')) {
      positions[payload.deviceId]['latitude'] = payload.latitude;
      positions[payload.deviceId]['longitude'] = payload.longitude;
      positions[payload.deviceId]['deviceTime'] = payload.deviceTime;
    }
    if (payload.hasOwnProperty('altitude')) {
      positions[payload.deviceId]['altitude'] = payload.altitude;
    }
    if (payload.hasOwnProperty('course')) {
      positions[payload.deviceId]['course'] = payload.course;
    }
    if (payload.hasOwnProperty('speed')) {
      positions[payload.deviceId]['speed'] = payload.speed;
    }

    //------------  Attributes -------
    if (payload.hasOwnProperty('batteryLevel')) {
      positions[payload.deviceId]['attributes']['batteryLevel'] = Math.round(Number.parseFloat(payload.batteryLevel));
    }
    if (payload.hasOwnProperty('gimbal')) {
      positions[payload.deviceId]['attributes']['gimbal'] = [
        round(payload.gimbal.x, 1),
        round(payload.gimbal.y, 1),
        round(payload.gimbal.z, 1),
      ];
    }
    if (payload.hasOwnProperty('obstacle_info')) {
      positions[payload.deviceId]['attributes']['obstacle_info'] = [
        round(payload.obstacle_info?.down, 1),
        round(payload.obstacle_info?.front, 1),
        round(payload.obstacle_info?.right, 1),
        round(payload.obstacle_info?.back, 1),
        round(payload.obstacle_info?.left, 1),
        round(payload.obstacle_info?.up, 1),
      ];
    }
    if (payload.hasOwnProperty('setHome')) {
      positions[payload.deviceId]['attributes']['home'] = [
        positions[payload.deviceId].latitude,
        positions[payload.deviceId].longitude,
        positions[payload.deviceId].altitude,
      ];
    }
    if (payload.hasOwnProperty('localposition')) {
      positions[payload.deviceId]['attributes']['localposition'] = [
        round(payload.localposition.x, 2),
        round(payload.localposition.y, 2),
        round(payload.localposition.z, 2),
      ];
      positions[payload.deviceId]['deviceTime'] = new Date().toISOString();
    }
    if (payload.hasOwnProperty('armState')) {
      positions[payload.deviceId]['attributes']['armState'] = payload.armState;
      positions[payload.deviceId]['attributes']['navState'] = payload.navState;
      positions[payload.deviceId]['attributes']['failsafe'] = payload.failsafe;
      positions[payload.deviceId]['attributes']['flightCheck'] = payload.flightCheck;
    }
    if (payload.hasOwnProperty('commandAck')) {
      positions[payload.deviceId]['attributes']['commandAck'] = payload.commandAck;
      positions[payload.deviceId]['attributes']['resultCmdAck'] = payload.resultCmdAck;
    }
    if (payload.hasOwnProperty('uav_state')) {
      positions[payload.deviceId]['attributes']['mission_state'] = payload.mission_state;
      positions[payload.deviceId]['attributes']['wp_reached'] = payload.wp_reached;
      positions[payload.deviceId]['attributes']['uav_state'] = payload.uav_state;
      positions[payload.deviceId]['attributes']['landed_state'] = this.convert_landed_state(
        payload.protocol,
        payload.landed_state
      );
    } else {
      if (payload.hasOwnProperty('landed_state')) {
        positions[payload.deviceId]['attributes']['uav_state'] = this.convert_landed_state(
          payload.protocol,
          payload.landed_state
        );
        positions[payload.deviceId]['attributes']['landed_state'] = this.convert_landed_state(
          payload.protocol,
          payload.landed_state
        );
      }
    }
    if (payload.hasOwnProperty('sensors_humidity')) {
      if (payload.sensors_humidity[0]) {
        positions[payload.deviceId]['attributes']['MIC_1'] = payload.sensors_humidity[0];
      }
      if (payload.sensors_humidity[1]) {
        positions[payload.deviceId]['attributes']['MIC_2'] = payload.sensors_humidity[1];
      }
      if (payload.sensors_humidity[2]) {
        positions[payload.deviceId]['attributes']['MIC_3'] = payload.sensors_humidity[2];
      }
      if (payload.sensors_humidity[3]) {
        positions[payload.deviceId]['attributes']['Metano'] = payload.sensors_humidity[3];
      }
      if (payload.sensors_humidity[4]) {
        positions[payload.deviceId]['attributes']['Alcohol'] = payload.sensors_humidity[4];
      }
      if (payload.sensors_humidity[5]) {
        positions[payload.deviceId]['attributes']['CO'] = payload.sensors_humidity[5];
      }
    }

    if (payload.hasOwnProperty('threat')) {
      if (payload.threat == 2) {
        if (positions[payload.deviceId]['attributes']['alarm'] != 'threat') {
          eventsController.addEvent({
            type: 'warning',
            deviceId: payload.deviceId,
            attributes: {
              action: 'Thread',
              message: 'Detected a threat',
              positions: { latitude: payload.latitude, longitude: payload.longitude, altitude: payload.altitude },
            },
          });
        }
        positions[payload.deviceId]['attributes']['alarm'] = 'threat';
      } else {
        if (payload.threat == 3) {
          if (positions[payload.deviceId]['attributes']['alarm'] != 'confirm') {
            eventsController.addEvent({
              type: 'warning',
              deviceId: payload.deviceId,
              attributes: {
                action: 'Thread',
                message: 'Confirmed a threat',
                positions: { latitude: payload.latitude, longitude: payload.longitude, altitude: payload.altitude },
              },
            });
          }
          positions[payload.deviceId]['attributes']['alarm'] = 'confirm';
        } else {
          positions[payload.deviceId]['attributes']['alarm'] = 'None';
        }
      }
    }

    return positions[payload.deviceId];
  }
  static convert_landed_state(protocol, landed_state) {
    let state_px4_stol = ['UNDEFINED', 'ON GROUND', 'IN AIR', 'TAKEOFF', 'LANDING'];
    let state_dji = ['STOPED', 'ON GROUND', 'IN AIR'];
    if (protocol == 'dji') {
      return state_dji[landed_state];
    }
    if (protocol == 'catec') {
      return landed_state;
    }
    return state_px4_stol[landed_state];
  }
}
