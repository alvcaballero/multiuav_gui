import { map } from 'zod';
import { eventsController } from '../controllers/events.js';
import { round } from '../common/utils.js';
const positions = {};
const history = {};
const camera = {};

export class positionsModel {
  static async getAll(query) {
    if (query) {
      console.log(query);
      if (Array.isArray(query)) {
        const asArray = Object.entries(positions);
        const filtered = asArray.filter(([key, value]) => query.some((element) => key == element));
        return Object.fromEntries(filtered);
      }
      if (!isNaN(query)) {
        return positions[query] ? { id: positions[query] } : {};
      }
    }
    return positions;
  }

  static async getByDeviceId(deviceId) {
    return positions[deviceId];
  }
  static async getCamera() {
    return camera;
  }
  static updateCamera(payload) {
    camera[payload.deviceId] = payload;
  }
  static removePosition({ id }) {
    delete positions[id];
    console.log(positions);
  }
  static async updatePosition(payload) {
    if (payload === null) {
      // console.log('payload null');
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
          wp_reached: 0,
          uav_state: 'OK',
          landed_state: 'Ready',
          alarm: 'UNDEFINED',
        },
      };
    }
    //positions[payload.deviceId]['serverTime'] = new Date().toISOString();

    if (payload.hasOwnProperty('latitude')) {
      //positions[payload.deviceId]["deviceId"] = payload.deviceId;
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
      //positions[payload.deviceId]['attributes']['protocol'] = payload.protocol;
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
      //positions[payload.deviceId]['attributes']['threat'] = payload.threat;
      if (payload.threat == 2) {
        if (positions[payload.deviceId]['attributes']['alarm'] != 'threat') {
          eventsController.addEvent({
            type: 'warning',
            eventTime: new Date().toISOString(),
            deviceId: payload.deviceId,
            attributes: {
              message: 'Threat detected',
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
              eventTime: new Date().toISOString(),
              deviceId: payload.deviceId,
              attributes: {
                message: 'Threat confirmed',
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
