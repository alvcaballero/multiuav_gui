import { DevicesModel } from '../models/devices.js';
import { positionsModel } from '../models/positions.js';
import { eventsModel } from '../models/events.js';
import { response } from 'express';
export class websocketController {
  static async init() {
    const devices = await DevicesModel.getAll();
    const positions = await positionsModel.getAll();
    const server = await DevicesModel.serverStatus();
    //console.log('devices');
    //console.log(devices);
    let response = JSON.stringify({
      positions: Object.values(positions),
      server: { rosState: server.state },
      devices: Object.values(devices),
    });
    //console.log('init');
    //console.log(response);
    return response;
  }

  static async update() {
    let currentsocket = {};
    const positions = await positionsModel.getAll();
    const currentevent = await eventsModel.getall();
    if (Object.values(positions).length) {
      currentsocket['positions'] = Object.values(positions);
    }
    //if (Object.values(data.state.camera).length) {
    //  currentsocket['camera'] = Object.values(data.state.camera);
    //}

    if (Object.values(currentevent).length) {
      //console.log(currentevent);
      currentsocket['events'] = Object.values(currentevent);
      data.clearEvents({ eventId: Object.keys(currentevent) });
    }
    console.log('update');
    //console.log(currentsocket);
    return JSON.stringify(currentsocket);
  }

  static async updateserver() {
    const devices = await DevicesModel.getAll();
    const server = await DevicesModel.serverStatus();

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
