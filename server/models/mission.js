import { DevicesModel } from '../models/devices.js';
import { readYAML, getDatetime } from '../common/utils.js';
import { SFTPClient } from '../common/SFTPClient.js';
import * as fs from 'fs';
import { exec } from 'child_process';
import { WebsocketManager } from '../WebsocketManager.js';
import { missionSMModel } from './missionSM.js';

const devices_init = readYAML('../config/devices/devices_init.yaml');
const Mission = { id: -1, uav: [], status: 'init', route: [], initTime: null }; // current mission // id , status (init, planing, doing, finish,time inti, time_end))
const sftconections = {}; // manage connection to drone
const filestodownload = []; //manage files that fail download// list of objects,with name, and fileroute, number of try.

export class missionModel {
  static getmission() {
    console.log('Get mission');
    return Mission;
  }
  static sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
  static async sendTask({ mission_id, objectivo, loc, meteo }) {
    console.log('command-sendtask');
    // plainig
    let mission = this.planning({ mission_id, objectivo, loc, meteo });
    // remplase planing with mission default
    console.log(mission.route[0]['wp'][0]['pos']);
    if (mission_id >= 1400) {
      mission = readYAML(`../config/mission/mission_${mission_id}.yaml`);
    } else {
      mission = readYAML('../config/mission.yaml');
    }

    var ws = new WebsocketManager(null, '/api/socket');
    ws.broadcast(JSON.stringify({ mission: { name: 'name', mission: mission } }));

    Mission['id'] = mission_id;
    Mission['uav'].push({ uav: 'uav_15', status: 'init' });
    Mission['route'] = mission.route;
    Mission['status'] = 'command';
    Mission['initTime'] = new Date();

    missionSMModel.createActorMission(mission_id); // create actor
    // init actor
    // poner uno de load plaing, for waiting the planer is load
    // send planing

    // generate response
    let response = { uav: mission.route[0].uav, points: [], status: 'OK' };
    response.points = mission.route[0].wp.map((wp) => {
      return [wp.pos[0], wp.pos[1], wp.pos[2]];
    });

    //let myresponse = { response };
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

  //En lista archivos y directorios, los directorios no se pueden descargar da error, se uede probar errores
  // crear funcion que solo enliste los archivos
  // hacer que las funciones devuelvan errores  para poder manejarlos
  // when in gcs are a file with the same name of file to download

  static async updateFiles({ id_uav, id_mission }) {
    console.log('update files ' + id_uav + '-' + id_mission);
    let listimages = [];
    let uav_name = DevicesModel.get_device_ns(id_uav);
    let mydevice = devices_init.init.find(({ name }) => name === uav_name);
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
      let dir = `/home/grvc/GCS_media/mission_${Mission['id']}/${uav_name}`;
      let dir2 = `mission_${Mission['id']}/${uav_name}`;

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
        let response = await fetch('http://localhost:1234/token/provide/RESISTO-API', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
          },
          body: new URLSearchParams({
            username: 'drone',
            password: 'F1PpE9V!E#Pwz8k53b7b',
          }),
        });
        if (response.ok) {
          let myresponse = await response.json();
          if (myresponse.access_token) {
            access_token = myresponse.access_token;
          }
        }
        if (access_token) {
          let sendresponse = await fetch('http://localhost:1234/resultado_mision', {
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

  // de momento lee la fecha mayour
  // puede hacer para que la lista de archivos se ordene de menor a mayour,
  // el primero que supere la fecha sera la carpeta, de la mission
  // this is importan when use database for get mission diferent
  static async showFiles({ id_uav, id_mission }) {
    console.log('show files ' + id_uav + '-' + id_mission);
    let uav_name = DevicesModel.get_device_ns(id_uav);
    let mydevice = devices_init.init.find(({ name }) => name === uav_name);
    const client = new SFTPClient();

    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    const parsedURL = new URL(myurl);
    const port = parsedURL.port || 22;
    const { host, username, password } = parsedURL;
    await client.connect({ host, port, username, password });
    //let listfiles = await client.listFiles('./uav_media/',"^mission_");// ^mission_2\d{3}-(0[1-9]|1[012])-(0[1-9]|[12][0-9]|3[01])_([0-2][0-9]):([0-5][0-9])
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
