import { DevicesModel } from '../models/devices.js';



export class websocketController {
  static async init() {
    return     JSON.stringify({
        positions: Object.values(data.state.positions),
        camera: Object.values(data.state.camera),
        server: { rosState: DevicesModel.serverStatus()},
        devices: Object.values(DevicesModel.getAll()),
      })
  }

  static async update() {
    let currentsocket = {};
    if (Object.values(data.state.positions).length) {
      currentsocket['positions'] = Object.values(data.state.positions);
    }
    if (Object.values(data.state.camera).length) {
      currentsocket['camera'] = Object.values(data.state.camera);
    }
    let currentevent = data.state.events;
    if (Object.values(currentevent).length) {
      console.log(currentevent);
      currentsocket['events'] = Object.values(currentevent);
      data.clearEvents({ eventId: Object.keys(currentevent) });
    }
    return JSON.stringify(currentsocket)
  }

  static async updateserver() {
    return       JSON.stringify({
        server: { rosState:  DevicesModel.serverStatus().state },
        devices: Object.values(DevicesModel.getAll()),
      })
  }

}
