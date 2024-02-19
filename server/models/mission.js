import dotenv from 'dotenv';
import * as fs from 'fs';
import { exec } from 'child_process';

import { DevicesModel } from '../models/devices.js';
import { readYAML, getDatetime } from '../common/utils.js';
import { SFTPClient } from '../common/SFTPClient.js';
import { WebsocketManager } from '../WebsocketManager.js';
import { missionSMModel } from './missionSM.js';
import { planningModel } from './planning.js';

const devices_init = readYAML('../config/devices/devices_init.yaml');
const Mission = {}; // current mission // id , status (init, planing, doing, finish,time inti, time_end))
const requestPlanning = {};
const sftconections = {}; // manage connection to drone
const filestodownload = []; //manage files that fail download// list of objects,with name, and fileroute, number of try.

dotenv.config();

const filesPath = process.env.Files_PATH ?? '/home/grvc/work/GCS_media/';
const planningServer = process.env.PLANNING_SERVER ?? 'http://127.0.0.1:8004/';
const extUrl = process.env.EXT_APP_url ?? 'http://127.0.0.1:8004';
const extUser = process.env.EXT_APP_user ?? 'user';
const extPwd = process.env.EXT_APP_pwd ?? 'password';

export class missionModel {
  static getmission(id) {
    console.log('Get mission' + id);
    if (id) {
      return Mission[id].mission;
    }
    return Mission[id].mission;
  }

  static sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  static async sendTask({ mission_id, objetivo, loc, meteo }) {
    console.log('command-sendtask');

    let myTask = {};
    myTask.id = mission_id;
    myTask.name = 'automatic';
    myTask.loc = loc;
    myTask.objetivo = objetivo;
    myTask.bases = planningModel.getBases();
    myTask.meteo = meteo;
    let devices = await DevicesModel.getAll();
    console.log(devices);
    let basesettings = planningModel.getBasesSettings();
    console.log(basesettings);
    myTask.settings = basesettings.map((setting) => {
      let mySetting = JSON.parse(JSON.stringify(setting));
      let myDevice = Object.values(devices).find((device) => device.id == setting.devices.deviceId);
      mySetting.devices.deviceId = myDevice.name;
      mySetting.devices.category = myDevice.category;
      return mySetting;
    });
    console.log(myTask.settings);

    console.log(myTask);
    const response1 = await fetch(`${planningServer}/mission_request`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(myTask),
    });
    if (response1.ok) {
      const response2 = await response1.json();
      console.log(response2);

      requestPlanning[mission_id] = {};
      requestPlanning[mission_id]['count'] = 0;
      requestPlanning[mission_id]['interval'] = setInterval(() => {
        console.log('response Planning' + mission_id);
        this.fetchPlanning(mission_id);
      }, 5000);
    } else {
      throw Error(await response.text());
    }
    return { response: myTask, status: 'OK' };
  }

  static async fetchPlanning(mission_id) {
    console.log('Fetch planning ' + mission_id);
    const response = await fetch(`${planningServer}/get_plan?IDs=${mission_id}`);
    if (response.ok) {
      const data = await response.json();
      console.log(data);
      console.log('response to check planning');
      if (Array.isArray(data.results) && data.results.length > 0) {
        if (data.results[0].hasOwnProperty('route')) {
          clearInterval(requestPlanning[mission_id]['interval']);
          requestPlanning[mission_id]['count'] = 10;
          this.setMission(data.results[0]);
          this.initMission(mission_id, data.results[0]);
        }
      }
    } else {
      throw Error(await response.text());
    }
    requestPlanning[mission_id]['count'] = requestPlanning[mission_id]['count'] + 1;
    if (requestPlanning[mission_id]['count'] > 3) {
      clearInterval(requestPlanning[mission_id]['interval']);
    }
  }

  static async initMission(missionId, mission) {
    const listUAV = mission.route.map((route) => DevicesModel.getByName(route.uav).id);
    Mission[missionId] = {
      id: missionId,
      uav: listUAV,
      status: 'init',
      initTime: null,
      mission: mission,
    };
    listUAV.forEach((uavId) => {
      missionSMModel.createActorMission(uavId, missionId);
    });
  }

  static async setMission(mission) {
    console.log('send mission');
    console.log(mission);

    var ws = new WebsocketManager(null, '/api/socket');
    ws.broadcast(JSON.stringify({ mission: { ...mission, name: 'name' } }));

    return { response: mission, status: 'OK' };
  }

  static planning({ mission_id, objectivo, loc, meteo }) {
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
    return mission;
  }

  /*En lista archivos y directorios, los directorios no se pueden descargar da error, se uede probar errores
   / crear funcion que solo enliste los archivos
   / hacer que las funciones devuelvan errores  para poder manejarlos
   / when in gcs are a file with the same name of file to download
   */
  static async updateFiles({ id_uav, id_mission }) {
    console.log('update files ' + id_uav + '-' + id_mission);
    let listimages = [];
    let mydevice = DevicesModel.getById(id_uav);
    const client = new SFTPClient();

    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    const parsedURL = new URL(myurl);
    const port = parsedURL.port || 22;
    const { host, username, password } = parsedURL;
    await client.connect({ host, port, username, password });
    let listfolders = await client.listFiles('./uav_media/', '^mission_', 'd', true); // que devuelva un  lista de objetos con la fecha de creacion
    let namefolder = null;
    for (const myfiles of listfolders) {
      var matches = myfiles.match(/^mission_(\d{4})-(\d{2})-(\d{2})_(\d{2}):(\d{2})$/);
      if (matches) {
        console.log(matches[0]);
        namefolder = myfiles;
        let folderdate = myfiles.slice(8).replace('_', 'T') + ':00';
        let currentdate = new Date(folderdate);
        console.log(
          folderdate + ' date  ' + currentdate.toJSON() + ' gcs=' + Mission['initTime'].toJSON()
        );
        if (Mission['initTime'] && currentdate > Mission['initTime']) {
          console.log('folder mayor');
        }
        break;
      }
    }
    if (namefolder) {
      let dir = `${filesPath}mission_${Mission['id']}/${mydevice.name}`;
      let dir2 = `mission_${Mission['id']}/${mydevice.name}`;

      if (!fs.existsSync(dir)) {
        console.log('no exist ' + dir);
        fs.mkdirSync(dir, { recursive: true });
      }

      let listfiles = await client.listFiles('./uav_media/' + namefolder + '/', '.jpg$', '-', true);
      console.log(listfiles);
      let downloadOk = true;
      for (let myfile of listfiles) {
        console.log('download file' + myfile);
        let response = await client.downloadFile(
          `./uav_media/${namefolder}/${myfile}`,
          `${dir}/${myfile}`
        );
        console.log(response);
        filestodownload.push({
          source: `./uav_media/${namefolder}/${myfile}`,
          dist: `${dir}/${myfile}`,
          ref: `${dir2}/${myfile}`,
          status: response.status,
        });
        if (!response.status) {
          downloadOk = false;
        }
      }
      //* Close the connection
      await client.disconnect();
      console.log(filestodownload);

      if (downloadOk) {
        listimages = filestodownload.map((image) => image.ref);
        console.log(listimages);
        //source activate my_env && python my_script.py
        //python3 /home/arpa/work/px4/multiuav_gui/scripts/utils/circleOpencv.py  "/home/arpa/GCS_media/mission_1/uav_15/DJI_20231124131954_0001_THRM.jpg" "/home/arpa/GCS_media/mission_1/uav_15/DJI_20231124131954_0001_THRM_process.jpg"
        let listimagestoprocess = filestodownload.filter((image) => image.dist.includes('THRM'));
        console.log('last images process');
        console.log(listimagestoprocess);
        for (let imagetoprocess of listimagestoprocess) {
          listimages.push(imagetoprocess.ref.slice(0, -4) + '_process.jpg');
          exec(
            `source activate DJIThermal && python3 /home/grvc/work/px4/multiuav_gui/scripts/utils/processThemalimages.py "${
              imagetoprocess.dist
            }" "${imagetoprocess.dist.slice(0, -4)}_process.jpg"  && conda deactivate`,
            (error, stdout, stderr) => {
              if (error) {
                console.log(`error: ${error.message}`);
                return;
              }
              if (stderr) {
                console.log(`stderr: ${stderr}`);
                return;
              }
              console.log(`stdout: ${stdout}`);
            }
          );
        }
        await this.sleep(5000);

        let access_token = null;
        console.log(extUrl + '---' + extUser + '---' + extPwd + '--');
        let response = await fetch(`${extUrl}/token`, {
          method: 'POST',
          body: `username=${extUser}&password=${extPwd}`,
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        });
        if (response.ok) {
          let myresponse = await response.json();
          console.log('ok test ok');
          console.log(myresponse);
          if (myresponse.access_token) {
            access_token = myresponse.access_token;
          }
        } else {
          throw new Error(`${response.status} ${response.statusText}`);
        }
        if (access_token) {
          let sendresponse = await fetch(`${extUrl}/resultado_mision`, {
            method: 'POST',
            headers: {
              Authorization: `Bearer ${access_token}`,
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              mission_id: Mission['id'],
              resolution_code: '0',
              resultados: listimages,
            }),
          });
          if (sendresponse.ok) {
            let command = await sendresponse.json();
            console.log(command);
          } else {
            throw new Error(sendresponse.status);
          }
        }
      }
    }

    return listimages;
  }

  static async testAPP() {
    console.log('test external app');
    let access_token = null;
    console.log(extUrl + '---' + extUser + '---' + extPwd + '--');
    let response = await fetch(`${extUrl}/token`, {
      method: 'POST',
      body: `username=${extUser}&password=${extPwd}`,
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    });
    if (response.ok) {
      let myresponse = await response.json();
      console.log('ok test ok');
      console.log(myresponse);

      if (myresponse.access_token) {
        access_token = myresponse.access_token;
      }
    } else {
      console.log(`${response.status} ${response.statusText}`);
    }
    return { status: true };
  }

  // de momento lee la fecha mayour
  // puede hacer para que la lista de archivos se ordene de menor a mayour,
  // el primero que supere la fecha sera la carpeta, de la mission
  // this is importan when use database for get mission diferent
  static async showFiles({ id_uav, id_mission }) {
    console.log('show files ' + id_uav + '-' + id_mission);
    let mydevice = DevicesModel.getById(id_uav);
    const client = new SFTPClient();

    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    const parsedURL = new URL(myurl);
    const port = parsedURL.port || 22;
    const { host, username, password } = parsedURL;
    await client.connect({ host, port, username, password });
    let listfiles = await client.listFiles('./uav_media/', '^mission_', 'd', true); // que devuelva un  lista de objetos con la fecha de creacion
    let namefolder = null;
    for (const myfiles of listfiles) {
      var matches = myfiles.match(/^mission_(\d{4})-(\d{2})-(\d{2})_(\d{2}):(\d{2})$/);
      if (matches) {
        console.log(matches[0]);
        namefolder = myfiles;
        let folderdate = myfiles.slice(8).replace('_', 'T') + ':00';
        let currentdate = new Date(folderdate);
        console.log(
          folderdate + ' date  ' + currentdate.toJSON() + ' gcs=' + Mission['initTime'].toJSON()
        );
        if (Mission['initTime'] && currentdate > Mission['initTime']) {
          console.log('folder mayor');
        }
        break;
      }
    }
    if (namefolder) {
      listfiles = await client.listFiles('./uav_media/' + namefolder + '/', '.jpg$', '-', true);
    }
    //* Close the connection
    await client.disconnect();
    return listfiles;
  }
  static async listFiles({ id_uav, id_mission }) {
    console.log('devices acction ' + id_uav);
    return [];
  }
}
