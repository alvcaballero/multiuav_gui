import * as fs from 'fs';
import { dateString, GetLocalTime, readDataFile } from '../common/utils.js';
import { SFTPClient } from '../common/SFTPClient.js';
import { FTPClient } from '../common/FTPClient.js';
import { devicesController } from '../controllers/devices.js';
import { filesPath, filesData } from '../config/config.js';
import { getMetadata, ProcessThermalImage } from './ProcessFile.js';
import { missionController } from '../controllers/mission.js';
import sequelize from '../common/sequelize.js';

/* files:
/    id
/    routeId
/    missionId
/    deviceId
/    name
/    source : {url, type}
/    path : path in server /mission_id/uav_name/
/    path2 : path in the drone or gcs 
/    status: FILE_STATUS
/    date:
/    attributes: metadata of the file 
*/

export const FILE_STATUS = Object.freeze({
  NO_DOWNLOAD: 0,
  DOWNLOAD: 1,
  PROCESS: 2,
  FAIL: 3,
  ERROR: 4,
  OK: 5,
});

const filesSetup = readDataFile('../config/devices/devices.yaml');
const downloadQueue = []; // manage files to download
const processQueue = []; // manage files to process

export class filesModel {
  static async getFiles({ id, deviceId, missionId, routeId }) {
    if (deviceId) {
      return await sequelize.models.File.findAll({ where: { deviceId: deviceId } });
    }
    if (missionId) {
      return await sequelize.models.File.findAll({ where: { missionId: missionId } });
    }
    if (routeId) {
      return await sequelize.models.File.findAll({ where: { routeId: routeId } });
    }
    if (id) {
      return sequelize.models.File.findOne({ where: { id: id } });
    }
    return await sequelize.models.File.findAll();
  }

  static async addFile({
    name,
    routeId,
    missionId,
    deviceId,
    status = FILE_STATUS.NO_DOWNLOAD,
    type,
    path,
    path2,
    source,
    date = new Date(),
    attributes = {},
  }) {
    const newFile = {
      name,
      routeId,
      missionId,
      deviceId,
      status,
      type,
      path,
      path2,
      source,
      date,
      attributes,
    };
    const myfile = await sequelize.models.File.create(newFile); // save to database
    return myfile;
  }

  static async editFile({ id, status, attributes }) {
    let file = await sequelize.models.File.findOne({ where: { id: id } });
    if (!file) {
      return null;
    }
    if (status) file.status = status;
    if (attributes) file.attributes = attributes;
    if (status == FILE_STATUS.OK) {
      let myfiles = await this.getFiles({ routeId: file.routeId });
      let allFilesOk = myfiles.every((file) => file.status == FILE_STATUS.OK || file.status == FILE_STATUS.ERROR);
      if (allFilesOk) {
        await MissionController.endRouteUAV(file.missionId, file.deviceId);
      }
    }
    await file.save();
    return file;
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

  static checkFileRoute(path) {
    let dir = filesPath + path.replaceAll('-', '/');
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
    console.log(`list files ${path} ${lastFolder} ${filterFolder} ${filterFile}`);
    let listFolder = [];
    let myfiles = [];
    let myfolders = await client.listFiles(path, filterFolder, 'd', true);
    if (lastFolder == true) {
      if (myfolders.length > 0) {
        listFolder.push(myfolders[0]);
      }
    } else {
      listFolder = myfolders;
      myfiles = await client.listFiles(path, filterFile, '-', true);
    }
    let listFiles = myfiles.map((file) => `${path}${file}`);
    for (const myfolder of listFolder) {
      let Filesfolder = await this.mylistFiles(client, `${path}${myfolder}/`, false, '', filterFile);
      listFiles = listFiles.concat(Filesfolder);
    }
    return listFiles;
  }

  /* 
  / Show list of files in the drone, from folder uav_media,
  */

  static async showFiles({ uavId, missionId, initTime, index = 0 }) {
    let mydevice = await devicesController.getAccess(uavId);
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
    console.log(myconfig);
    let params = this.paramsConnection({ mydevice: mydevice.files[index] });
    const client = params.protocol == 'ftp:' ? new FTPClient() : new SFTPClient();
    let status = await client.connect(params);
    if (!status) {
      console.log('cant connect to device ');
      return [];
    }
    console.log(`config files ${myconfig.path} ${myconfig.type}`);
    listFiles = await this.mylistFiles(client, myconfig.path, myconfig.type == 'lastFolder' ? true : false);

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
    console.log(`update files api call ${uavId} ${routeId} ${missionId} ${initTime} ${index}`);

    let mydevice = await devicesController.getAccess(uavId);
    let myconfig = filesSetup.files.default;
    let listFiles = [];

    if (!mydevice.hasOwnProperty('files') && mydevice.files.length == 0) {
      console.log('UAV no have files setup ');
      return [];
    }

    const deviceFile = JSON.parse(JSON.stringify(mydevice.files[index]));
    console.log(deviceFile);

    if (deviceFile.hasOwnProperty('type')) {
      myconfig = filesSetup.files.hasOwnProperty(deviceFile.type)
        ? filesSetup.files[deviceFile.type]
        : filesSetup.files.default;
    }

    let params = this.paramsConnection({ mydevice: deviceFile });
    const client = params.protocol == 'ftp:' ? new FTPClient() : new SFTPClient();

    let status = await client.connect(params);
    if (!status) {
      console.log('cant connect to device ');
      return [];
    }

    console.log(`config files ${myconfig.path} ${myconfig.type}`);

    let pathFolder = myconfig.path;
    if (myconfig.type == 'specific') {
      let myInitTime = dateString(GetLocalTime(initTime)).replace(/-|:|\s/g, '_');
      pathFolder = `${myconfig.path}mission_${myInitTime}/`;
    }

    listFiles = await this.mylistFiles(client, pathFolder, myconfig.type == 'lastFolder');

    await client.disconnect();

    if (listFiles.length == 0) {
      console.log('no files to download');
      return [];
    }

    let dir = `${filesPath}mission_${missionId}/${mydevice.name}`;
    if (!fs.existsSync(dir)) {
      console.log('no exist ' + dir);
      fs.mkdirSync(dir, { recursive: true });
    }

    for (let myfile of listFiles) {
      let createFile = await this.addFile({
        routeId: routeId,
        missionId: missionId,
        deviceId: uavId,
        name: `${myfile.split('/').at(-1)}`,
        path: `mission_${missionId}/${mydevice.name}/`,
        source: deviceFile,
        path2: myfile,
      });
      downloadQueue.push(createFile.id);
    }

    this.downloadFiles();

    if (+index + 1 < mydevice.files.length) {
      this.updateFiles({ uavId, missionId, initTime, index: index + 1 });
    }

    return true;
  }

  static async downloadFiles2(client, url, fileId, remove = false) {
    let myfile = await this.getFiles({ id: fileId });

    let response = await client.downloadFile(myfile.path2, `${filesPath}${myfile.path}${myfile.name}`);
    if (response.status) {
      await this.editFile({ id: fileId, status: FILE_STATUS.DOWNLOAD });
      processQueue.push(fileId);
      if (remove) {
        await client.deleteFile(myfile.path2);
      }
    } else {
      await this.editFile({ id: fileId, status: FILE_STATUS.FAIL });
    }
    const checkUrl = await this.getFiles({ id: downloadQueue[0] });
    if (downloadQueue.length > 0 && checkUrl.source.url === url) {
      await this.downloadFiles2(client, url, downloadQueue.shift(), remove);
    }
  }

  static async downloadFiles() {
    console.log('download files');
    if (downloadQueue.length == 0) {
      return;
    }
    let myFileId = downloadQueue.shift();
    let myfile = await this.getFiles({ id: myFileId });

    let mydevice = await devicesController.getAccess(myfile.deviceId);
    let myconfig = filesSetup.files.default;

    if (!mydevice.hasOwnProperty('files') && mydevice.files.length == 0) {
      console.log('UAV no have files setup ');
      return [];
    }

    if (myfile?.source?.hasOwnProperty('type')) {
      myconfig = filesSetup.files.hasOwnProperty(myfile.source.type)
        ? filesSetup.files[myfile.source.type]
        : filesSetup.files.default;
    }

    let params = this.paramsConnection({ mydevice: { url: myfile.source.url } });
    const client = params.protocol == 'ftp:' ? new FTPClient() : new SFTPClient();

    let status = await client.connect(params);
    if (!status) {
      console.log('cant connect to device ');
      return [];
    }

    await this.downloadFiles2(client, myfile.source.url, myfile.id, myconfig.delete);

    client.disconnect();

    this.processFiles();

    if (downloadQueue.length > 0) {
      this.downloadFiles();
    }

    return;
  }

  /*
   *
   */
  static async processFiles() {
    console.log('process file');
    if (processQueue.length == 0) {
      return;
    }
    let myFileId = processQueue.shift();
    let myfile = await this.getFiles({ id: myFileId });
    if (myfile.name.includes('THRM') && !myfile.name.includes('process')) {
      let response = await ProcessThermalImage(
        `${filesPath}${myfile.path}${myfile.name}`,
        `${filesPath}${myfile.path}${myfile.name.slice(0, -4)}_process.jpg`
      );
      if (response) {
        let createFile = await this.addFile({
          routeId: myfile.routeId,
          missionId: myfile.missionId,
          deviceId: myfile.deviceId,
          name: `${myfile.name.split('.')[0]}_process.jpg`,
          path: myfile.path,
          source: 'GCS',
          path2: `${myfile.path}${myfile.name}`,
          status: FILE_STATUS.DOWNLOAD,
          date: myfile.date,
        });
        processQueue.push(createFile.id);
        await this.editFile({ id: myFileId, status: FILE_STATUS.OK });
      } else {
        await this.editFile({ id: myFileId, status: FILE_STATUS.ERROR });
      }
    }
    try {
      let attributes = await getMetadata(`${filesPath}${myfile.path}${myfile.name}`);
      await this.editFile({ id: myFileId, status: FILE_STATUS.OK, attributes });
    } catch (e) {
      console.log('error metadata');
      await this.editFile({ id: myFileId, status: FILE_STATUS.ERROR, attributes: {} });
    }
    // end process call other function for continuos the process of state machine
    if (processQueue.length > 0) this.processFiles();
  }

  static async ProcessThermalImages(src) {
    if (src.length == 0) return false;
    console.log('process thermal images');
    console.log(src);
    for (const file of src) {
      if (file.includes('THRM') || file.includes('.tiff')) {
        let response = await ProcessThermalImage(`${file}`, `${file.split('.')[0]}_process.jpg`);
      }
    }
    return true;
  }

  static async listFiles({ uavId, missionId }) {
    console.log('devices acction ' + uavId);
    return [];
  }
}
