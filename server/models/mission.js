import { ros } from '../models/ros.js';
import { DevicesModel } from '../models/devices.js';
import { readYAML, getDatetime } from '../common/utils.js';
import ROSLIB from 'roslib';
import { SFTPClient } from '../common/SFTPClient.js';
import * as fs from 'fs';
import { WebsocketManager } from '../WebsocketManager.js';

const devices_init = readYAML('../config/devices/devices_init.yaml');
const mission = {} // current mission // id , status (init, planing, doing, finish,time inti, time_end))
const sftconections = {};// manage connection to drone 
const filestodownload = {}; //manage files that fail download// list of objects,with name, and fileroute, number of try.



export class missionModel {
  static getmission() {
    console.log('Get mission');
    return [];
  }

  static async sendTask({ loc }) {
    console.log('command-sendtask');
    let uav = 'uav_15';

    let home = [37.134092, -6.472401, 50];
    let reqRoute = Object.values(loc);
    let mission = {
      version: '3',
      route: [{ name: 'datetime', uav: uav, wp: [] }],
      status: 'OK',
    };
    let response = { uav: uav, points: [], status: 'OK' };
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
    var ws = new WebsocketManager(null,'/api/socket')
    ws.broadcast(JSON.stringify({ mission: { name: 'name', mission: mission } }));
    let myresponse = { response };
    return myresponse;
  }

  static async updateFiles({ id_uav, id_mission }) {
    console.log('update files ' + id_uav +'-'+id_mission);
    let uav_name = DevicesModel.get_device_ns(id_uav);
    let mydevice = devices_init.init.find(({ name }) => name === uav_name);
    const client = new SFTPClient();

    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    const parsedURL = new URL(myurl);
    const port = parsedURL.port || 22;
    const { host, username, password } = parsedURL;
    await client.connect({ host, port, username, password });
    let listfiles = await client.listFiles('./uav_media/',"^mission_");// que devuelva un  lista de objetos con la fecha de creacion
    let myfiledownload = 'mission_2023-11-27_13:00'
    //En lista archivos y directorios, los directorios no se pueden descargar da error, se uede probar errores
    // crear funcion que solo enliste los archivos
    // hacer que las funciones devuelvan errores  para poder manejarlos
    listfiles = await client.listFiles(`./uav_media/${myfiledownload}`);
    let dir =`/home/arpa/GCS_media/mission_1/${uav_name}`
    if (!fs.existsSync(dir)){
      console.log("no exist " + dir)
      fs.mkdirSync(dir, { recursive: true });
    }
    for (let myfile of listfiles) {
      console.log(myfile);
      await client.downloadFile(`./uav_media/${myfiledownload}/${myfile}`, `${dir}/${myfile}`);
    }
    //* Close the connection
    await client.disconnect();
    return listfiles;
  }

  static async showFiles({ id_uav, id_mission }) {
    console.log('show files ' + id_uav +'-'+id_mission);
    let uav_name = DevicesModel.get_device_ns(id_uav);
    let mydevice = devices_init.init.find(({ name }) => name === uav_name);
    const client = new SFTPClient();

    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    const parsedURL = new URL(myurl);
    const port = parsedURL.port || 22;
    const { host, username, password } = parsedURL;
    await client.connect({ host, port, username, password });
    let listfiles = await client.listFiles('./uav_media/',"^mission_");// que devuelva un  lista de objetos con la fecha de creacion
    listfiles = await client.listFiles('./uav_media/');
    listfiles = await client.listFiles('./uav_media/','','d');
    console.log(listfiles)
    listfiles = await client.listFiles('./uav_media/mission_2023-11-27_13:00','','all',true);
    console.log(listfiles)
    listfiles = await client.listFiles('./uav_media/mission_2023-11-27_13:00');
    listfiles = await client.listFiles('./uav_media/mission_2023-11-27_13:00',".jpg$");
    //* Close the connection
    await client.disconnect();
    return listfiles;
  }
  static async listFiles({ id_uav, id_mission }) {
    console.log('devices acction ' + id_uav);
    return [];
  }
}
