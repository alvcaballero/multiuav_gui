import { ros } from '../models/ros.js';
import { DevicesModel } from '../models/devices.js';
import { readYAML, getDatetime } from '../common/utils.js';
import ROSLIB from 'roslib';
import { SFTPClient } from '../common/SFTPClient.js';
import * as fs from 'fs';
import { WebsocketManager } from '../WebsocketManager.js';

const devices_init = readYAML('../config/devices/devices_init.yaml');
const Mission = {id:-1,uav:[],status:'init',route:[],initTime:null} // current mission // id , status (init, planing, doing, finish,time inti, time_end))
const sftconections = {};// manage connection to drone 
const filestodownload = []; //manage files that fail download// list of objects,with name, and fileroute, number of try.



export class missionModel {
  static getmission() {
    console.log('Get mission');
    return Mission;
  }

  static async sendTask({ misision_id,objectivo,loc,meteo }) {
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
    Mission['id']= misision_id;
    Mission['uav'].push({uav:'uav_15',status:'init'})
    Mission['route']= mission.route;
    Mission['status']= 'command';
    Mission['initTime']=new Date();
    let myresponse = { response };
    return myresponse;
  }

    //En lista archivos y directorios, los directorios no se pueden descargar da error, se uede probar errores
    // crear funcion que solo enliste los archivos
    // hacer que las funciones devuelvan errores  para poder manejarlos

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
    let listfiles = await client.listFiles('./uav_media/',"^mission_",'d',true);// que devuelva un  lista de objetos con la fecha de creacion
    let namefolder = null;
    for (const myfiles of listfiles) {
      var matches = myfiles.match(/^mission_(\d{4})-(\d{2})-(\d{2})_(\d{2}):(\d{2})$/);
      if (matches ){
        console.log(matches[0])
        namefolder =myfiles;
        let folderdate = (myfiles.slice(8)).replace('_','T')+':00';
        let currentdate = new Date(folderdate);
        console.log(folderdate+" date  "+currentdate.toJSON())
        if(Mission['initTime'] && currentdate > Mission['initTime']){
          console.log('folder mayor')
        }
        break;
      } 
    }
    if(namefolder){
      let dir =`/home/arpa/GCS_media/mission_${Mission['id']}/${uav_name}`

      if (!fs.existsSync(dir)){
        console.log("no exist " + dir)
        fs.mkdirSync(dir, { recursive: true });
      }

      listfiles = await client.listFiles('./uav_media/'+namefolder+'/',".jpg$",'-',true)
      downloadOk= true
      for (let myfile of listfiles) {
        console.log(myfile);
        let response = await client.downloadFile(`./uav_media/${myfiledownload}/${myfile}`, `${dir}/${myfile}`);
        console.log(response)
        filestodownload.push({source:`./uav_media/${myfiledownload}/${myfile}`,dist:`${dir}/${myfile}`,status:response.status})
        if(!response.status){
          downloadOk = false;
        }
      }
      if(downloadOk){ 
        let sendresponse = await fetch('http://localhost:8000/resultado_mision', {
          method: 'POST',
          headers: {
            Accept: 'application/json',
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ mission_id: Mission['id'],resolution_code:'0',resultado: filestodownload }),
        });
        if (sendresponse.ok) {
          let command = await sendresponse.json();
        } else {
          throw new Error(sendresponse.status);
          console.log('uboi')
        }
      }
      
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
    //let listfiles = await client.listFiles('./uav_media/',"^mission_");// ^mission_2\d{3}-(0[1-9]|1[012])-(0[1-9]|[12][0-9]|3[01])_([0-2][0-9]):([0-5][0-9])
    let listfiles = await client.listFiles('./uav_media/',"^mission_",'d',true);// que devuelva un  lista de objetos con la fecha de creacion
    let namefolder = null;
    for (const myfiles of listfiles) {
      var matches = myfiles.match(/^mission_(\d{4})-(\d{2})-(\d{2})_(\d{2}):(\d{2})$/);
      if (matches ){
        console.log(matches[0])
        namefolder =myfiles;
        let folderdate = (myfiles.slice(8)).replace('_','T')+':00';
        let currentdate = new Date(folderdate);
        console.log(folderdate+" date  "+currentdate.toJSON())
        if(Mission['initTime'] && currentdate > Mission['initTime']){
          console.log('folder mayor')
        }
        break;
      } 
    }
    if(namefolder){
      listfiles = await client.listFiles('./uav_media/'+namefolder+'/',".jpg$",'-',true)
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
