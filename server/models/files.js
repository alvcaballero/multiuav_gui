//example that a men manage databases
import * as fs from 'fs';
//import { exec } from 'child_process';

import { dateString, addTime, GetLocalTime, readJSON, readYAML } from '../common/utils.js';
import { SFTPClient } from '../common/SFTPClient.js';
import { FTPClient } from '../common/FTPClient.js';
import { DevicesController } from '../controllers/devices.js';
import { filesPath } from '../config/config.js';
import { getMetadata, ProcessThermalImage } from './ProcessFile.js';

/* files:
/     
/    status: 0: no download, 1: download, 2: process, 3: fail download , 4: error, 5: ok, 
*/

const files = readJSON('../data/files.json');
const filesSetup = readYAML('../config/devices/devices.yaml');
const downloadQueue = []; // manage files to download
const processQueue = []; // manage files to process

export class filesModel {
  static getFiles({ id, deviceId, missionId, routeId }) {
    if (deviceId) {
      return Object.values(files).filter((word) => word.deviceId == deviceId);
    }
    if (missionId) {
      return Object.values(files).filter((word) => word.missionId == missionId);
    }
    if (routeId) {
      return Object.values(files).filter((word) => word.routeId == routeId);
    }
    if (id) {
      return files[id];
    }
    return Object.values(files);
  }

  static addFile({
    routeId,
    missionId,
    deviceId,
    name,
    route,
    source,
    path2,
    status = 0,
    date = new Date(),
    attributes = {},
  }) {
    let id = Math.max(...Object.keys(files).map((key) => Number(key))) + 1;
    files[id] = {
      id,
      routeId,
      missionId,
      deviceId,
      name,
      route,
      source,
      path2,
      status,
      date,
      attributes,
    };
    return files[id];
  }

  static editFile({ id, status, attributes }) {
    if (!files[id]) {
      return null;
    }
    if (status) files[id].status = status;
    if (attributes) files[id].attributes = attributes;
    return files[id];
  }
  /*
  / read all files in the gcs in folder GCS_MEDIA , and return a list of files
  */

  static readGCSFiles() {
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

  /*
  / donwload a file from gcs, if the file exist return the path, if not return null
  */

  static donwload(path) {
    let dir = filesPath + path.replaceAll('-', '/');
    console.log(dir);
    if (!fs.existsSync(dir)) {
      console.log('no exist ' + dir);
      return null;
    }
    return dir;
  }

  /* 
  / Manage params to connect to the drone, and return the params to connect to the drone
  */

  static paramsConnection({ mydevice }) {
    console.log(mydevice);
    let myurl = '';
    myurl = mydevice.url;
    const parsedURL = new URL(myurl);
    let port = parsedURL.port || 21;
    const { hostname, username, password, protocol } = parsedURL;
    console.log(`host: ${hostname}:${port}, user: ${username}, pwd: ${password}, protocol: ${protocol}`);
    return { host: hostname, port, username, password, protocol };
  }

  // filter file '^mission_'
  // filter file '.jpg$'

  static async mylistFiles(client, path, lastFolder = false, filterFolder = '', filterFile = '') {
    let listFolder = [];
    let myfiles = [];
    let myfolders = await client.listFiles(path, filterFolder, 'd', true);
    if (lastFolder == true) {
      listFolder = myfolders;
    } else {
      if (myfolders.length > 0) {
        listFolder.push(myfolders[0]);
      }
      myfiles = await client.listFiles(path, filterFile, '-', true);
    }
    let listFiles = myfiles.map((file) => `${path}${file}`);
    for (const myfolder of listFolder) {
      let Filesfolder = await this.mylistFiles(client, `${path}${myfolder}/`, false, '', filterFile);
      listFiles = listFiles.concat(Filesfolder);
    }
    return listFiles;
  }

  static async showFiles({ uavId, missionId, initTime, index = 0 }) {
    let mydevice = await DevicesController.getAccess(uavId);
    let myconfig = filesSetup.files.default;
    let listFiles = [];
    console.log(`show files api call ${uavId} ${missionId} ${initTime} ${index}`);
    if (!mydevice.hasOwnProperty('files') && mydevice.files.length == 0) {
      console.log('UAV no have files setup ');
      return [];
    }
    console.log(mydevice.files[index]);

    if (mydevice.files[index].hasOwnProperty('type')) {
      myconfig = filesSetup.files.hasOwnProperty(mydevice.files[index].type)
        ? filesSetup.files[mydevice.files[index].type]
        : filesSetup.files.default;
    }
    let params = this.paramsConnection({ mydevice: mydevice.files[index] });
    const client = params.protocol == 'ftp:' ? new FTPClient() : new SFTPClient();
    let status = await client.connect(params);
    if (!status) {
      console.log('cant connect to device ');
      return [];
    }
    console.log(`config files ${myconfig.path} ${myconfig.type}`);
    listFiles = await this.mylistFiles(client, myconfig.path, myconfig.type == 'lastFolder');

    await client.disconnect();

    if (+index + 1 < mydevice.files.length) {
      let otherFiles = await this.showFiles({ uavId, missionId, initTime, index: index + 1 });
      console.log('other files' + otherFiles.length);
      otherFiles.forEach((file) => {
        listFiles.push(file);
      });
    }

    return listFiles;
  }

  /*
   / Update files in the drone, from folder uav_media, and download to the server (GCS)
   / if the file is a thermal image, process the image and return the metadata
   / return a list of files, and a list of metadata 
   */
  static async updateFiles(uavId, missionId, routeId, initTime, index = 0) {
    let mydevice = await DevicesController.getAccess(uavId);
    let myconfig = filesSetup.files.default;
    let listFiles = [];

    console.log(`update files api call ${uavId} ${missionId} ${initTime} ${index}`);
    if (!mydevice.hasOwnProperty('files') && mydevice.files.length == 0) {
      console.log('UAV no have files setup ');
      return [];
    }
    console.log(mydevice.files[index]);

    if (mydevice.files[index].hasOwnProperty('type')) {
      myconfig = filesSetup.files.hasOwnProperty(mydevice.files[index].type)
        ? filesSetup.files[mydevice.files[index].type]
        : filesSetup.files.default;
    }

    let params = this.paramsConnection({ mydevice: mydevice.files[index] });
    const client = params.protocol == 'ftp:' ? new FTPClient() : new SFTPClient();

    let status = await client.connect(params);
    if (!status) {
      console.log('cant connect to device ');
      return [];
    }

    console.log(`config files ${myconfig.path} ${myconfig.type}`);

    let myInitTime = dateString(GetLocalTime(initTime)).replace(/-|:|\s/g, '_');
    let nameFolder = 'mission_' + myInitTime;

    listFiles = await this.mylistFiles(client, myconfig.path, myconfig.type == 'lastFolder');

    await client.disconnect();

    if (listFiles.length == 0) {
      console.log('no files to download');
      return [];
    }

    if (!fs.existsSync(dir)) {
      console.log('no exist ' + dir);
      fs.mkdirSync(dir, { recursive: true });
    }

    let dir = `${filesPath}mission_${missionId}/${mydevice.name}`;
    let dir2 = `mission_${missionId}/${mydevice.name}`;

    if (nameFolder) {
      for (let myfile of listFiles) {
        let createFile = this.addFile({
          routeId: routeId,
          missionId: missionId,
          deviceId: uavId,
          name: `${myfile.split('/').at(-1)}`,
          route: `${dir2}/`,
          source: mydevice.files[index],
          path2: myfile,
        });
        downloadQueue.push(createFile.id);
      }
    }
    this.downloadFiles();

    if (+index + 1 < mydevice.files.length) {
      this.updateFiles({ uavId, missionId, initTime, index: index + 1 });
    }

    return true;
  }

  static async downloadFiles2(client, url, fileId, remove = false) {
    let myfile = files[fileId];

    let response = await client.downloadFile(myfile.path2, `${myfile.route}${myfile.name}`);
    if (response) {
      this.editFile({ id: fileId, status: 1 });
      processQueue.push(fileId);
      if (remove) {
        await client.deleteFile(myfile.path2);
      }
    } else {
      this.editFile({ id: fileId, status: 3 });
    }

    if (downloadQueue.length > 0 && files[downloadQueue[0]].source === url) {
      await downloadFiles2(client, url, downloadQueue.shift(), remove);
    }
  }

  static async downloadFiles() {
    console.log('download files');
    if (downloadQueue.length == 0) {
      return;
    }
    let myFileId = downloadQueue.shift();
    let myfile = files[myFileId];

    let mydevice = await DevicesController.getAccess(myfile.deviceId);
    let myconfig = filesSetup.files.default;

    if (!mydevice.hasOwnProperty('files') && mydevice.files.length == 0) {
      console.log('UAV no have files setup ');
      return [];
    }

    if (mydevice.files[index].hasOwnProperty('type')) {
      myconfig = filesSetup.files.hasOwnProperty(myfile.source.type)
        ? filesSetup.files[myfile.source.type]
        : filesSetup.files.default;
    }

    let params = this.paramsConnection({ mydevice: myfile.source.url });
    const client = params.protocol == 'ftp:' ? new FTPClient() : new SFTPClient();

    let status = await client.connect(params);
    if (!status) {
      console.log('cant connect to device ');
      return [];
    }

    await downloadFiles2(client, myfile.source.url, myfile.id, myconfig.delete);

    client.disconnect();

    if (downloadQueue.length > 0) {
      this.downloadFiles();
    }

    this.processFiles();
    return;
  }

  static async processFiles() {
    console.log('process images');
    if (processQueue.length == 0) {
      return;
    }
    let myFileId = processQueue.shift();
    let myfile = files[myFileId];
    if (myfile.name.includes('THRM') == 'thermal' && !myfile.name.includes('process')) {
      let response = await ProcessThermalImage(
        `${myfile.route}${myfile.name}`,
        `${myfile.route}${myfile.name.slice(0, -4)}_process.jpg`
      );
      if (response) {
        let createFile = this.addFile({
          routeId: myfile.routeId,
          missionId: myfile.missionId,
          deviceId: myfile.deviceId,
          name: `${myfile.name.slice(0, -4)}_process.jpg`,
          route: myfile.route,
          source: 'GCS',
          path2: `${myfile.route}${myfile.name}`,
          status: 5,
          date: myFile.date,
        });
        processQueue.push(createFile.id);
        this.editFile({ id: myFileId, status: 5 });
      } else {
        this.editFile({ id: myFileId, status: 4 });
      }
    }
    let attributes = await getMetadata(`${myfile.route}${myfile.name}`);
    this.editFile({ id: myFileId, status: 5, attributes });
    if (processQueue.length > 0) {
      this.processFiles();
    }
    // end process call other function for continuos the process of state machine
  }

  static async listFiles({ uavId, missionId }) {
    console.log('devices acction ' + uavId);
    return [];
  }
}
