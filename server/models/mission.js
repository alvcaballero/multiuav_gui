import { ros } from '../models/ros.js';
import { DevicesModel } from '../models/devices.js';
import { readYAML, getDatetime } from '../common/utils.js';
import ROSLIB from 'roslib';
import { SFTPClient } from '../common/SFTPClient.js';

const devices_init = readYAML('../config/devices/devices_init.yaml');
const sftconections = {};

export class missionModel {
  static getmission() {
    console.log('Get mission');
    return [];
  }

  static async sendTask({ loc }) {
    console.log('command-sendtask');
    let uav = 'uav_1';
    //let home = [37.193736, -6.702947, 50];
    let home = [37.134092, -6.472401, 50];
    let reqRoute = Object.values(loc);
    let mission = {
      version: '3',
      route: [{ name: 'datetime', uav: 'uav_1', wp: [] }],
      status: 'OK',
    };
    let response = { uav: 'uav_1', points: [], status: 'OK' };
    response.points.push(home);
    for (let i = 0; i < reqRoute.length; i = i + 1) {
      let wp_len = reqRoute[i].length;
      for (let j = 0; j < wp_len; j = j + 1) {
        if (j == 0) {
          response.points.push([reqRoute[i][j]['lat'], reqRoute[i][j]['lon'], 50]);
        }
        response.points.push([reqRoute[i][j]['lat'], reqRoute[i][j]['lon'], 30]);
        if (j == +wp_len + -1) {
          response.points.push([reqRoute[i][j]['lat'], reqRoute[i][j]['lon'], 50]);
        }
      }
    }

    response.points.push(home);
    mission.route[0]['wp'] = response.points.map((element) => {
      return { pos: element };
    });
    console.log(mission.route[0]['wp'][0]['pos']);
    //wss.clients.forEach(function each(ws) {
    //  ws.send(JSON.stringify({ mission: { name: 'name', mission: mission } }));
    //});
    let myresponse = { response };
    return myresponse;
  }

  static async updateFiles({ id_uav, id_mission }) {
    console.log('update files ' + id_uav);
    let uav_name = DevicesModel.get_device_ns(id_uav);
    let mydevice = devices_init.init.find(({ name }) => name === uav_name);
    const client = new SFTPClient();

    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    const parsedURL = new URL(myurl);
    const port = parsedURL.port || 22;
    const { host, username, password } = parsedURL;
    await client.connect({ host, port, username, password });
    let listfiles = await client.listFiles('.');
    //* Close the connection
    await client.disconnect();
    return listfiles;
  }

  static async showFiles({ id_uav, id_mission }) {
    console.log('show files ' + id_uav);
    let uav_name = DevicesModel.get_device_ns(id_uav);
    let mydevice = devices_init.init.find(({ name }) => name === uav_name);
    const client = new SFTPClient();

    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    const parsedURL = new URL(myurl);
    const port = parsedURL.port || 22;
    const { host, username, password } = parsedURL;
    await client.connect({ host, port, username, password });
    let listfiles = await client.listFiles('.');
    //* Close the connection
    await client.disconnect();
    return listfiles;
  }
  static async listFiles({ id_uav, id_mission }) {
    console.log('devices acction ' + id_uav);
    return [];
  }
}
