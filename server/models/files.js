import * as fs from 'fs';
import { missionFolderStamp } from '../common/utils.js';
import { SFTPClient } from '../common/SFTPClient.js';
import { FTPClient } from '../common/FTPClient.js';
import { devicesController } from '../controllers/devices.js';
import { missionDataPath } from '../config/config.js';
import { getMetadata, ProcessThermalImage } from './ProcessFile.js';
import { missionController } from '../controllers/mission.js';
import sequelize from '../common/sequelize.js';
import { missionLogger as logger } from '../common/logger.js';

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
    try {
      const myfile = await sequelize.models.File.create(newFile);
      return myfile;
    } catch (error) {
      logger.error(
        `addFile FK constraint failed missionId=${missionId} routeId=${routeId} deviceId=${deviceId}: ${error.message}`
      );
      return null;
    }
  }

  static async editFile({ id, status, attributes }) {
    let file = await sequelize.models.File.findOne({ where: { id: id } });
    if (!file) {
      return null;
    }
    if (status) file.status = status;
    if (attributes) file.attributes = attributes;
    await file.save();
    if (status == FILE_STATUS.OK) {
      let myfiles = await this.getFiles({ routeId: file.routeId });
      let allFilesOk = myfiles.every((file) => file.status == FILE_STATUS.OK || file.status == FILE_STATUS.ERROR);
      if (allFilesOk) {
        await missionController.endRouteUAV(file.missionId, file.deviceId);
      }
    }
    return file;
  }

  /*
  / read all files in the gcs in folder GCS_MEDIA , and return a list of files
  */

  static readGCSFiles() {
    let response = [];
    let firstFiles = fs.readdirSync(missionDataPath, { withFileTypes: true });
    let missionFolder = firstFiles.filter((myroute) => myroute.isDirectory());
    for (let mymission of missionFolder) {
      var stats = fs.statSync(missionDataPath + mymission.name + '/');
      logger.debug(`last time create in seconds: ${stats.mtime}`);
      let secondFiles = fs.readdirSync(missionDataPath + mymission.name + '/', { withFileTypes: true });
      let uavfolder = secondFiles.filter((myroute) => myroute.isDirectory());
      for (let myuav of uavfolder) {
        let thirdFiles = fs.readdirSync(missionDataPath + mymission.name + '/' + myuav.name + '/', {
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
    let dir = missionDataPath + path.replaceAll('-', '/');
    if (!fs.existsSync(dir)) {
      logger.warn(`file path does not exist: ${dir}`);
      return null;
    }
    return dir;
  }

  /* 
  / Manage params to connect to the drone, and return the params to connect to the drone
  */

  static paramsConnection({ mydevice }) {
    logger.debug(`paramsConnection device: ${JSON.stringify(mydevice)}`);
    let myurl = '';
    myurl = mydevice.url;
    const parsedURL = new URL(myurl);
    let port = parsedURL.port || 21;
    const { hostname, username, password, protocol } = parsedURL;
    logger.debug(`host: ${hostname}:${port}, user: ${username}, protocol: ${protocol}`);
    return { host: hostname, port, username, password, protocol };
  }

  // filter file '^mission_'
  // filter file '.jpg$'

  static async mylistFiles(client, path, lastFolder = false, filterFolder = '', filterFile = '') {
    logger.debug(`list files ${path} lastFolder=${lastFolder} filterFolder=${filterFolder} filterFile=${filterFile}`);
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

  static async showFiles({ uavId, missionId, initTime }) {
    logger.info(`show files api call ${uavId} ${missionId} ${initTime}`);
    const configs = await devicesController.getFilesConfig(uavId);
    if (configs.length === 0) {
      return [];
    }

    let listFiles = [];
    for (const myconfig of configs) {
      logger.debug(`myconfig: ${JSON.stringify(myconfig)}`);
      let params = this.paramsConnection({ mydevice: myconfig });
      const client = params.protocol == 'ftp:' ? new FTPClient() : new SFTPClient();
      let status = await client.connect(params);
      if (!status) {
        logger.warn('cant connect to device');
        continue;
      }
      logger.debug(`config files ${myconfig.path} ${myconfig.type}`);
      const sourceFiles = await this.mylistFiles(client, myconfig.path, myconfig.type == 'lastFolder');
      await client.disconnect();
      listFiles = listFiles.concat(sourceFiles);
    }

    return listFiles;
  }

  /*
   / Update files in the drone, from folder uav_media, and download to the server (GCS)
   / if the file is a thermal image, process the image and return the metadata
   / return a list of files, and a list of metadata 
   */
  static async updateFiles(uavId, missionId, routeId, initTime) {
    logger.info(`update files api call uavId=${uavId} routeId=${routeId} missionId=${missionId}`);

    const mydevice = await devicesController.getAccess(uavId);
    const configs = await devicesController.getFilesConfig(uavId);
    if (configs.length === 0) {
      return [];
    }

    let queued = false;
    for (const myconfig of configs) {
      logger.debug(`config files ${myconfig.path} ${myconfig.type}`);
      let params = this.paramsConnection({ mydevice: myconfig });
      const client = params.protocol == 'ftp:' ? new FTPClient() : new SFTPClient();

      let status = await client.connect(params);
      if (!status) {
        logger.warn('cant connect to device');
        continue;
      }

      let pathFolder = myconfig.path;
      if (myconfig.type == 'specific') {
        // The remote folder name is derived from the mission's real initTime in the
        // DB (the same UTC instant sent to the UAV as init_date), NOT from whatever
        // the client passed in — a client-supplied local-time string would never
        // match the folder the onboard computer actually created.
        const mission = await missionController.getMissionRoute(missionId);
        const dbInitTime = mission?.initTime ?? initTime;
        let myInitTime = missionFolderStamp(dbInitTime);
        pathFolder = `${myconfig.path}mission_${myInitTime}/`;
      }

      const listFiles = await this.mylistFiles(client, pathFolder, myconfig.type == 'lastFolder');
      await client.disconnect();

      if (listFiles.length == 0) {
        logger.warn('no files to download');
        continue;
      }

      let dir = `${missionDataPath}mission_${missionId}/${mydevice.name}`;
      if (!fs.existsSync(dir)) {
        logger.debug(`creating directory: ${dir}`);
        fs.mkdirSync(dir, { recursive: true });
      }

      for (let myfile of listFiles) {
        let createFile = await this.addFile({
          routeId: routeId,
          missionId: missionId,
          deviceId: uavId,
          name: `${myfile.split('/').at(-1)}`,
          path: `mission_${missionId}/${mydevice.name}/`,
          source: myconfig,
          path2: myfile,
        });
        if (createFile) {
          downloadQueue.push(createFile.id);
          queued = true;
        }
      }
    }

    if (queued) this.downloadFiles();

    return true;
  }

  static async downloadFiles2(client, url, fileId, remove = false) {
    let myfile = await this.getFiles({ id: fileId });

    let response = await client.downloadFile(myfile.path2, `${missionDataPath}${myfile.path}${myfile.name}`);
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
    logger.debug('download files');
    if (downloadQueue.length == 0) {
      return;
    }
    let myFileId = downloadQueue.shift();
    let myfile = await this.getFiles({ id: myFileId });

    // `source` holds the already-resolved files config (url + delete + ...) that
    // updateFiles stored — no need to re-read the YAML or re-resolve the preset.
    const myconfig = myfile?.source ?? {};

    let params = this.paramsConnection({ mydevice: { url: myconfig.url } });
    const client = params.protocol == 'ftp:' ? new FTPClient() : new SFTPClient();

    let status = await client.connect(params);
    if (!status) {
      logger.warn('cant connect to device');
      return [];
    }

    await this.downloadFiles2(client, myconfig.url, myfile.id, myconfig.delete);

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
    logger.debug('process file');
    if (processQueue.length == 0) {
      return;
    }
    let myFileId = processQueue.shift();
    let myfile = await this.getFiles({ id: myFileId });
    if (myfile.name.includes('THRM') && !myfile.name.includes('process')) {
      let response = await ProcessThermalImage(
        `${missionDataPath}${myfile.path}${myfile.name}`,
        `${missionDataPath}${myfile.path}${myfile.name.slice(0, -4)}_process.jpg`
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
      let attributes = await getMetadata(`${missionDataPath}${myfile.path}${myfile.name}`);
      await this.editFile({ id: myFileId, status: FILE_STATUS.OK, attributes });
    } catch (e) {
      logger.error('error reading file metadata', e);
      await this.editFile({ id: myFileId, status: FILE_STATUS.ERROR, attributes: {} });
    }
    // end process call other function for continuos the process of state machine
    if (processQueue.length > 0) this.processFiles();
  }

  static async ProcessThermalImages(src) {
    if (src.length == 0) return false;
    logger.info(`process thermal images: ${JSON.stringify(src)}`);
    for (const file of src) {
      if (file.includes('THRM') || file.includes('.tiff')) {
        await ProcessThermalImage(`${file}`, `${file.split('.')[0]}_process.jpg`);
      }
    }
    return true;
  }

  static async listFiles({ uavId, missionId: _missionId }) {
    logger.debug(`listFiles for uavId ${uavId}`);
    return [];
  }
}
