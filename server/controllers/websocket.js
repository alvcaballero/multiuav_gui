import { DevicesModel } from '../models/devices.js';
import { rosModel } from '../models/ros.js';
import { positionsModel } from '../models/positions.js';
import { eventsModel } from '../models/events.js';
import { readYAML } from '../common/utils.js';
const init_mission_elements = readYAML('../config/elements/config.yaml');
export class websocketController {
  static async init() {
    const devices = await DevicesModel.getAll();
    const positions = await positionsModel.getAll();
    const server = await rosModel.serverStatus();
    let response = JSON.stringify({
      positions: Object.values(positions),
      server: { rosState: server.state },
      devices: Object.values(devices),
      markers: { bases: init_mission_elements.bases, elements: init_mission_elements.markers },
    });
    return response;
  }

  static async update() {
    let currentsocket = {};
    const positions = await positionsModel.getAll();
    const camera = await positionsModel.getCamera();
    const currentevent = await eventsModel.getall();
    if (Object.values(positions).length) {
      currentsocket['positions'] = Object.values(positions);
    }
    if (Object.values(camera).length) {
      currentsocket['camera'] = Object.values(camera);
    }

    if (Object.values(currentevent).length) {
      //console.log(currentevent);
      currentsocket['events'] = Object.values(currentevent);
      eventsModel.clearEvents({ eventId: Object.keys(currentevent) });
    }
    //console.log('update');
    //console.log(currentsocket);
    return JSON.stringify(currentsocket);
  }

  static async updateserver() {
    const devices = await DevicesModel.getAll();
    const server = await rosModel.serverStatus();

    //console.log('devices');
    //console.log(devices);
    let response = JSON.stringify({
      server: { rosState: server.state },
      devices: Object.values(devices),
    });
    //console.log('updateserver');
    //console.log(response);
    return response;
  }
}
