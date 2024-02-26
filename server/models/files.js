//example that a men manage databases
import * as fs from 'fs';
import { exec } from 'child_process';

import { dateString, addTime, GetLocalTime } from '../common/utils.js';

import { SFTPClient } from '../common/SFTPClient.js';
import { DevicesModel } from '../models/devices.js';

import dotenv from 'dotenv';

dotenv.config();

const sftconections = {}; // manage connection to drone
const filestodownload = []; //manage files that fail download// list of objects,with name, and fileroute, number of try.

const filesPath = process.env.Files_PATH ?? '/home/grvc/work/GCS_media/';
const ProcessImg = process.env.Preprocess_Img ?? false;
const ProcessSRC = process.env.Preprocess_src ?? '';

export class filesModel {
  static getfiles() {
    let response = [];
    let firstFiles = fs.readdirSync(filesPath, { withFileTypes: true });
    let missionFolder = firstFiles.filter((myroute) => myroute.isDirectory());
    for (let mymission of missionFolder) {
      var stats = fs.statSync(filesPath + mymission.name + '/');
      console.log('last time create in secons' + stats.mtime);
      let secondFiles = fs.readdirSync(filesPath + mymission.name + '/', { withFileTypes: true });
      let uavfolder = secondFiles.filter((myroute) => myroute.isDirectory());
      for (let myuav of uavfolder) {
        let thirdFiles = fs.readdirSync(filesPath + mymission.name + '/' + myuav.name + '/', {
          withFileTypes: true,
        });
        let uavfiles = thirdFiles.filter((myroute) => myroute.isFile());
        for (let myfile of uavfiles) {
          response.push(mymission.name + '/' + myuav.name + '/' + myfile.name);
        }
      }
    }
    return response;
  }
  static donwload(path) {
    let dir = filesPath + path.replaceAll('-', '/');
    console.log(dir);
    if (!fs.existsSync(dir)) {
      console.log('no exist ' + dir);
      return null;
    }
    // check if file exist
    return dir;
  }

  /* de momento lee la fecha mayour
  / puede hacer para que la lista de archivos se ordene de menor a mayour,
  / el primero que supere la fecha sera la carpeta, de la mission
  / this is importan when use database for get mission diferent
  */
  static async showFiles({ uavId, missionId, initTime }) {
    console.log('show files ' + uavId + '-' + missionId);
    let mydevice = DevicesModel.getAccess(uavId);
    console.log(mydevice);
    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    const parsedURL = new URL(myurl);
    const port = parsedURL.port || 22;
    const { host, username, password } = parsedURL;
    const client = new SFTPClient();
    await client.connect({ host, port, username, password });
    let listFiles = await client.listFiles('./uav_media/', '^mission_', 'd', true); // que devuelva un  lista de objetos con la fecha de creacion
    let nameFolder = null;
    for (const myfiles of listFiles) {
      var matches = myfiles.match(/^mission_(\d{4})-(\d{2})-(\d{2})_(\d{2}):(\d{2})$/);
      if (matches) {
        console.log(matches[0]);
        nameFolder = myfiles;
        let folderdate = myfiles.slice(8).replace('_', 'T') + ':00';
        let currentdate = new Date(folderdate);
        console.log(folderdate + ' date  ' + currentdate.toJSON() + ' gcs=' + initTime.toJSON());
        if (initTime && currentdate > initTime) {
          console.log('folder mayor');
        }
        break;
      }
    }
    if (nameFolder) {
      listFiles = await client.listFiles('./uav_media/' + nameFolder + '/', '.jpg$', '-', true);
    }
    //* Close the connection
    await client.disconnect();
    return listFiles;
  }

  /*
   / list files and directories, directory can't donwload make Error, 
   / crear funcion que solo enliste los archivos
   / hacer que las funciones devuelvan errores  para poder manejarlos
   / when in gcs are a file with the same name of file to download
   */
  static async updateFiles(uavId, missionId, initTime) {
    console.log('update files ' + uavId + '-' + missionId);
    let listImages = [];
    let mydevice = DevicesModel.getAccess(uavId);
    console.log(mydevice);
    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    if (!mydevice.hasOwnProperty('user')) {
      console.log('no have ftp ');
      return false;
    }
    const client = new SFTPClient();
    const parsedURL = new URL(myurl);
    const port = parsedURL.port || 22;
    const { host, username, password } = parsedURL;

    let isConnected = await client.connect({ host, port, username, password });
    if (!isConnected) {
      console.log('cant connect to device ');
      return listImages;
    }
    let listFolders = await client.listFiles('./uav_media/', '^mission_', 'd', true); // que devuelva un  lista de objetos con la fecha de creacion
    let nameFolder = null;
    console.log('select folder');
    let myInitTime = dateString(GetLocalTime(initTime)).replace(/-|:|\s/g, '_');

    for (const myfiles of listFolders) {
      /* var matches = myfiles.match(/^mission_(\d{4})_(\d{2})_(\d{2})_(\d{2})_(\d{2})$/);
     * console.log(matches);
      if (matches) {
        console.log('matches');
        nameFolder = myfiles;
        let folderdate = myfiles.slice(8).replace('_', 'T') + ':00';
        let currentdate = new Date(folderdate);
        console.log(folderdate + ' date  ' + currentdate.toJSON() + ' gcs=' + initTime.toJSON());
        if (initTime && currentdate > initTime) {
          console.log('folder mayor');
          break;
        }
      }
      */
      console.log([myfiles] + '' + 'mission_' + myInitTime);
      if (myfiles === 'mission_' + myInitTime) {
        nameFolder = myfiles;
      }
    }
    console.log('name folder ' + nameFolder);
    if (nameFolder) {
      let dir = `${filesPath}mission_${missionId}/${mydevice.name}`;
      let dir2 = `mission_${missionId}/${mydevice.name}`;

      if (!fs.existsSync(dir)) {
        console.log('no exist ' + dir);
        fs.mkdirSync(dir, { recursive: true });
      }

      let listFiles = await client.listFiles('./uav_media/' + nameFolder + '/', '.jpg$', '-', true);
      console.log(listFiles);
      let downloadOk = true;
      for (let myfile of listFiles) {
        console.log('download file' + myfile);
        let response = await client.downloadFile(
          `./uav_media/${nameFolder}/${myfile}`,
          `${dir}/${myfile}`
        );
        console.log(response);
        filestodownload.push({
          source: `./uav_media/${nameFolder}/${myfile}`,
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
        listImages = filestodownload.map((image) => image.ref);
        console.log(listImages);
        let listImagestoprocess = filestodownload.filter((image) => image.dist.includes('THRM'));
        if (ProcessImg && listImagestoprocess.length > 0) {
          let listThermal = this.ProcessThermalImages(listImagestoprocess);
          return listImages.concat(listThermal);
        }
      }
    }

    return listImages;
  }

  static async ProcessThermalImages(Images2process) {
    console.log('last images process');
    console.log(Images2process);
    const listImages = [];
    for (let image of Images2process) {
      listImages.push(image.ref.slice(0, -4) + '_process.jpg');
      exec(
        `conda run -n DJIThermal ${ProcessSRC} -i "${image.dist}" -o "${image.dist.slice(
          0,
          -4
        )}_process.jpg" `,
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
    return listImages;
  }

  static async listFiles({ uavId, missionId }) {
    console.log('devices acction ' + uavId);
    return [];
  }
}
