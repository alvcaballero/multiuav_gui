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
  PROCESS: 2, // For  thermal images, the processed image is created and metadata read. For other files, this state is skipped.
  FAIL: 3, // Download failed (connection, missing file, etc.) or metadata read failed on a real image. Aligns with ROUTE_STATUS.ERROR / MISSION_STATUS.ERROR.
  ERROR: 4, // Error Processing the file or reading metadata. Aligns with ROUTE_STATUS.ERROR / MISSION_STATUS.ERROR.
  // Whole pipeline finished for this file: downloaded, processed and metadata read
  // successfully. Aligns with ROUTE_STATUS.COMPLETED / MISSION_STATUS.COMPLETED.
  COMPLETED: 5,
  // Downloaded fine but metadata could not (or need not) be read — e.g. a video,
  // which sharp can't parse. NOT an error: the file is intact. Appended at the end
  // so existing numeric statuses already persisted in the DB keep their meaning.
  DOWNLOAD_NO_METADATA: 6,
});

// File-name classification. Every check is CASE-INSENSITIVE: camera vendors emit
// mixed cases (DJI → `.JPG`/`.MP4`/`THRM`, others may differ), so a case-sensitive
// match would silently miss files. Extensions we attempt to read image metadata
// from (sharp + exif); anything else (videos, rosbags, lidar, ...) is downloaded
// but skipped for metadata reading.
const METADATA_IMAGE_EXTS = ['.jpg', '.jpeg', '.png', '.tif', '.tiff', '.webp'];

const extOf = (name) => {
  if (!name) return '';
  const dot = name.lastIndexOf('.');
  return dot === -1 ? '' : name.slice(dot).toLowerCase();
};
const isImageFile = (name) => METADATA_IMAGE_EXTS.includes(extOf(name));
// Thermal images need the extra ProcessThermalImage pass. DJI tags them with a
// `THRM` marker in the name; `.tif/.tiff` are treated as thermal too. Matched
// case-insensitively so `.TIFF` / `thrm` variants are not missed.
const isThermalFile = (name) => {
  const lower = (name ?? '').toLowerCase();
  return lower.includes('thrm') || lower.endsWith('.tif') || lower.endsWith('.tiff');
};

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

  static async editFile({ id, status, attributes, errorMessage }) {
    let file = await sequelize.models.File.findOne({ where: { id: id } });
    if (!file) {
      return null;
    }
    if (status) file.status = status;
    if (attributes) file.attributes = attributes;
    if (errorMessage != null) file.errorMessage = errorMessage;
    await file.save();
    // Close the route once every file for it has reached a TERMINAL state. A file
    // never advances on its own from any of these, so leaving one out would keep the
    // route open forever whenever that outcome occurs:
    //   OK / DOWNLOAD_NO_METADATA → success (metadata read, or none to read)
    //   FAIL (download failed) / ERROR (metadata read failed on a real image) → failure
    const isTerminal = (s) =>
      s == FILE_STATUS.COMPLETED ||
      s == FILE_STATUS.DOWNLOAD_NO_METADATA ||
      s == FILE_STATUS.FAIL ||
      s == FILE_STATUS.ERROR;
    const isFailure = (s) => s == FILE_STATUS.FAIL || s == FILE_STATUS.ERROR;
    if (isTerminal(status)) {
      let myfiles = await this.getFiles({ routeId: file.routeId });
      let allFilesDone = myfiles.every((f) => isTerminal(f.status));
      if (allFilesDone) {
        // Surface any per-file failures on the route so the reason isn't buried in
        // the File rows only. DOWNLOAD_NO_METADATA is NOT a failure — a video with no
        // metadata is a successful download. The route still ends (endRouteUAV) — a
        // partial download is a finished-with-errors mission, not a stuck one.
        const failed = myfiles.filter((f) => isFailure(f.status));
        if (failed.length > 0) {
          const errorMessage = `${failed.length}/${myfiles.length} archivo(s) no se descargaron o procesaron correctamente`;
          logger.warn(`route ${file.routeId} finished with file errors — ${errorMessage}`);
          await missionController.editRoute({ id: file.routeId, errorMessage });
        }
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
    // Accumulate the reason each config produced no files so, if the WHOLE
    // download yields nothing, we can persist a descriptive errorMessage on the
    // route. Note: SFTPClient.listFiles swallows list errors and returns [], so
    // "remote folder missing / SFTP list failed" and "folder present but empty"
    // are indistinguishable here — both surface as an empty listing.
    const downloadIssues = [];
    for (const myconfig of configs) {
      logger.debug(`config files ${myconfig.path} ${myconfig.type}`);
      let params = this.paramsConnection({ mydevice: myconfig });
      const client = params.protocol == 'ftp:' ? new FTPClient() : new SFTPClient();

      let status = await client.connect(params);
      if (!status) {
        logger.warn('cant connect to device');
        downloadIssues.push(`no se pudo conectar a ${params.host ?? 'dispositivo'} (${myconfig.path})`);
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
        downloadIssues.push(`sin archivos en ${pathFolder}`);
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

    if (queued) {
      logger.info(`updateFiles: ${downloadQueue.length} file(s) queued for download (route=${routeId})`);
      this.downloadFiles();
    }

    // Nothing got queued for download: either every config failed to connect or
    // every remote folder came back empty. Persist the reason on the route so the
    // failure is visible in the DB/UI instead of only in the logs. Only when we
    // have a routeId to address (some callers invoke updateFiles route-less).
    if (!queued && routeId) {
      const errorMessage =
        downloadIssues.length > 0
          ? `No se descargaron archivos: ${downloadIssues.join('; ')}`
          : 'No se descargaron archivos de la misión';
      logger.warn(`updateFiles: no files downloaded for route ${routeId} — ${errorMessage}`);
      await missionController.editRoute({ id: routeId, errorMessage });
    }

    return true;
  }

  static async downloadFiles2(client, url, fileId, remove = false) {
    let myfile = await this.getFiles({ id: fileId });

    // Progress: how many remain queued AFTER this one (this file is already shifted
    // out of downloadQueue by the caller), so operators can follow the download.
    logger.info(`downloadFiles2: downloading ${myfile.name} (${downloadQueue.length} remaining in queue)`);

    let response = await client.downloadFile(myfile.path2, `${missionDataPath}${myfile.path}${myfile.name}`);
    if (response.status) {
      await this.editFile({ id: fileId, status: FILE_STATUS.DOWNLOAD });
      processQueue.push(fileId);
      if (remove) {
        await client.deleteFile(myfile.path2);
      }
    } else {
      // Persist WHY it failed so a not-downloaded file is traceable in the DB (the
      // File row already keeps its origin: source.url + path2 to retry later).
      const errorMessage = `Fallo al descargar desde ${myfile.path2}: ${response.data ?? 'error desconocido'}`;
      logger.warn(`downloadFiles2: ${errorMessage}`);
      await this.editFile({ id: fileId, status: FILE_STATUS.FAIL, errorMessage });
    }
    // Only continue on the same connection if the next queued file shares this
    // URL. Guard the queue first: an empty queue would make getFiles({id:
    // undefined}) fall through to findAll() (a wasted full-table scan whose
    // array result has no `.source`).
    if (downloadQueue.length > 0) {
      const checkUrl = await this.getFiles({ id: downloadQueue[0] });
      if (checkUrl?.source?.url === url) {
        await this.downloadFiles2(client, url, downloadQueue.shift(), remove);
      }
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
    if (isThermalFile(myfile.name) && !myfile.name.toLowerCase().includes('process')) {
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
        await this.editFile({ id: myFileId, status: FILE_STATUS.COMPLETED });
      } else {
        await this.editFile({ id: myFileId, status: FILE_STATUS.ERROR });
      }
    }
    // Only images carry readable metadata for our pipeline (sharp + exif). A video
    // downloaded fine but has no image metadata — don't even try sharp on it (it
    // throws "unsupported image format"), just mark it downloaded-without-metadata
    // so it counts as a successful, terminal file instead of a false ERROR.
    if (!isImageFile(myfile.name)) {
      logger.info(`processFiles: ${myfile.name} downloaded, metadata skipped (not an image)`);
      await this.editFile({ id: myFileId, status: FILE_STATUS.DOWNLOAD_NO_METADATA });
    } else {
      try {
        let attributes = await getMetadata(`${missionDataPath}${myfile.path}${myfile.name}`);
        const nMeasures = attributes?.measures?.length ?? 0;
        logger.info(
          `processFiles: metadata OK for ${myfile.name} ` +
            `(gps=${attributes?.latitude != null ? 'yes' : 'no'}, measures=${nMeasures})`
        );
        await this.editFile({ id: myFileId, status: FILE_STATUS.COMPLETED, attributes });
      } catch (e) {
        // A real image whose metadata can't be read (corrupt / unexpected format).
        // Persist the reason on the File so it's traceable, same as download errors.
        const errorMessage = `No se pudieron leer los metadatos del archivo: ${e.message}`;
        logger.error(`processFiles: ${errorMessage} (${myfile.name})`);
        await this.editFile({ id: myFileId, status: FILE_STATUS.ERROR, attributes: {}, errorMessage });
      }
    }
    // end process call other function for continuos the process of state machine
    if (processQueue.length > 0) this.processFiles();
  }

  static async ProcessThermalImages(src) {
    if (src.length == 0) return false;
    logger.info(`process thermal images: ${JSON.stringify(src)}`);
    for (const file of src) {
      if (isThermalFile(file)) {
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
