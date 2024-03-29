import { map } from 'zod';
import { DevicesModel } from '../models/devices.js';
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

  static async getDevice(deviceId) {
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
    //positions[payload.deviceId] = payload;
    DevicesModel.updatedevicetime(payload.deviceId);

    if (positions[payload.deviceId] === undefined) {
      positions[payload.deviceId] = {
        deviceId: payload.deviceId,
        accuracy: 0.0,
        attributes: {
          gimbal: [0, 0, 0],
          obstacle_info: [0, 0, 0, 0, 0, 0],
          takeoff_height: 400,
          mission_state: 'Ready',
          wp_reached: 0,
          uav_state: 'OK',
          landed_state: 'Ready',
          alarm: 'UNDEFINED',
        },
      };
    }

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
      positions[payload.deviceId]['attributes']['batteryLevel'] = payload.batteryLevel;
    }
    if (payload.hasOwnProperty('gimbal')) {
      positions[payload.deviceId]['attributes']['gimbal'] = [
        payload.gimbal.x.toFixed(),
        payload.gimbal.y.toFixed(),
        payload.gimbal.z.toFixed(),
      ];
    }
    if (payload.hasOwnProperty('obstacle_info')) {
      positions[payload.deviceId]['attributes']['obstacle_info'] = [
        payload.obstacle_info.down.toFixed(),
        payload.obstacle_info.front.toFixed(),
        payload.obstacle_info.right.toFixed(),
        payload.obstacle_info.back.toFixed(),
        payload.obstacle_info.left.toFixed(),
        payload.obstacle_info.up.toFixed(),
      ];
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

    if (payload.hasOwnProperty('threat')) {
      //positions[payload.deviceId]['attributes']['threat'] = payload.threat;
      if (payload.threat == true) {
        positions[payload.deviceId]['attributes']['alarm'] = 'threat';
      } else {
        positions[payload.deviceId]['attributes']['alarm'] = undefined;
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
