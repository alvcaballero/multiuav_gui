//example that a men manage databases
import * as fs from 'fs';
//import { exec } from 'child_process';
import child_process from 'child_process';
import util from 'util';
import sharp from 'sharp';
import exif from 'exif-reader';
import sequelize from '../common/sequelize.js';

const exec = util.promisify(child_process.exec);

import { dateString, addTime, GetLocalTime } from '../common/utils.js';
import { SFTPClient } from '../common/SFTPClient.js';
import { DevicesController } from '../controllers/devices.js';
import { MissionController } from '../controllers/mission.js';
import { filesPath, processThermalImg, processThermalsSrc } from '../config/config.js';

// const  files// id , route ,name, uav, date, attributes

const sftconections = {}; // manage connection to drone
const filestodownload = []; //manage files that fail download// list of objects,with name, and fileroute, number of try.

export class filesModel {
  static async getFiles({ deviceId, missionId, routeId }) {
    if (deviceId) {
      return await sequelize.models.File.findAll({ where: { deviceId: deviceId } });
    }
    if (missionId) {
      return await sequelize.models.File.findAll({ where: { missionId: missionId } });
    }
    if (routeId) {
      return await sequelize.models.File.findAll({ where: { routeId: routeId } });
    }
    return await sequelize.models.File.findAll();
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
  / Show list of files in the drone, from folder uav_media,
  */

  static async showFiles({ uavId, missionId, initTime }) {
    console.log('show files ' + uavId + '-' + missionId);
    let mydevice = await DevicesController.getAccess(uavId);
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
    await client.disconnect();
    return listFiles;
  }

  /*
   / Update files in the drone, from folder uav_media, and download to the server (GCS)
   / if the file is a thermal image, process the image and return the metadata
   / return a list of files, and a list of metadata 
   */
  static async updateFiles(uavId, missionId, routeId, initTime) {
    console.log('update files ' + uavId + '-' + missionId);
    let listImages = [];
    let metadataResponse = { value: {}, imageMetaData: [] };
    // get device information
    let mydevice = await DevicesController.getAccess(uavId);
    if (!mydevice.hasOwnProperty('user')) {
      console.log('GCS dont have access to download files using ftp ');
      return false;
    }
    // create a connection to the drone
    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    const client = new SFTPClient();
    const parsedURL = new URL(myurl);
    const port = parsedURL.port || 22;
    const { host, username, password } = parsedURL;
    let isConnected = await client.connect({ host, port, username, password });
    if (!isConnected) {
      console.log('cant connect to device ');
      return listImages;
    }
    // get list of folders in the drone and the folder that match with the mission
    // ********** -----    Do that name folder is the same that the mission ----- *********
    let listFolders = await client.listFiles('./uav_media/', '^mission_', 'd', true); // que devuelva un  lista de objetos con la fecha de creacion
    let nameFolder = null;
    console.log('select folder');
    let myInitTime = dateString(GetLocalTime(initTime)).replace(/-|:|\s/g, '_');
    for (const myFolders of listFolders) {
      if (myFolders === 'mission_' + myInitTime) {
        nameFolder = myFolders;
        console.log('find folder 1');
      }
    }
    console.log('find folder ' + nameFolder);

    if (nameFolder) {
      let dir = `${filesPath}mission_${missionId}/${mydevice.name}`;
      let dirRef = `mission_${missionId}/${mydevice.name}`;

      if (!fs.existsSync(dir)) {
        console.log('no exist ' + dir);
        fs.mkdirSync(dir, { recursive: true });
      }

      let listFiles = await client.listFiles('./uav_media/' + nameFolder + '/', '.jpg$', '-', true);
      console.log('list files in uav');
      console.log(listFiles);
      for (let myfile of listFiles) {
        await sequelize.models.Media.create({
          routeId: routeId,
          missionId: missionId,
          deviceId: uavId,
          name: myfile,
          status: 'init',
          path: `${dirRef}/`,
          pathDevice: `./uav_media/${nameFolder}/`,
          date: new Date(),
        });
      }

      for (let myfile of listFiles) {
        console.log('download file' + myfile);
        let response = await client.downloadFile(`./uav_media/${nameFolder}/${myfile}`, `${dir}/${myfile}`);
        console.log(response);
        if (response.status) {
          sequelize.models.Media.update({ status: 'Downloaded' }, { where: { name: myfile } });
        }
        if (!response.status) {
          sequelize.models.Media.update({ status: 'error' }, { where: { name: myfile } });
        }
      }
      await client.disconnect();

      ProcessFiles(uavId, missionId, routeId, dirRef);
    }
    console.log(listImages);
    return { files: listImages, data: [metadataResponse.value] };
  }
  /*
   *
   */
  static async processFiles(uavId, missionId, routeId) {
    let myFiles = await sequelize.models.File.findAll({ where: { deviceId: uavId, missionId: missionId } });
    let listImagestoprocess = myFiles.filter((image) => image.name.includes('THRM') && image.status == 'Downloaded');
    for (processImg of listImagestoprocess) {
      let processResult = await this.ProcessThermalImages(processImg);
      if (processResult.state) {
        sequelize.models.Media.create(processResult.image);
      }
    }
    for (file of myFiles) {
      let metadataResponse = await this.metaDataImage(file);
      if (metadataResponse.state) {
        sequelize.models.Media.update(
          { attributes: JSON.stringify(metadataResponse.attributes), status: 'finish' },
          { where: { name: file.name, missionId: file.missionId, deviceId: file.deviceId } }
        );
      }
    }
    sequelize.models.Mission.findAll({
      attributes: ['attributes'],
      where: { missionId: missionId, deviceId: uavId },
    }).then((files) => {
      let allFinish = files.every((file) => file.status == 'finish');
      let maxAttributes = null;
      if (allFinish) {
        for (file of files) {
          let myAttribute = JSON.parse(file);
          if (
            maxAttributes !== null &&
            myAttribute.hasOwnProperty('measures') &&
            Number(myAttribute.measures[0].value) > Number(maxAttributes.measures[0].value)
          ) {
            maxAttributes = myAttribute;
          }
        }
        sequelize.models.Route.update(
          { status: 'finish', attributes: maxAttributes },
          { where: { missionId: missionId, deviceId: deviceId } }
        );
      }
    });
  }

  /*
  / return the metadata from a image
  */

  static async testMetadata(srcImage) {
    console.log(' test image');
    let metadata = await sharp(srcImage).metadata();
    let response1 = exif(metadata.exif);
    console.log(response1.GPSInfo.GPSLatitude[2] + '+' + response1.GPSInfo.GPSLongitude[2]);
    console.log(response1.GPSInfo);
    console.log(response1.Image.DateTime);

    let mystring = response1.Photo.UserComment.toString('utf8').replace(/\u0000/g, '');
    let response = JSON.parse(mystring.slice(7).trim());
    console.log(response);
    console.log(mystring);

    console.log(response);

    return response;
  }

  /*
  / return metadata from a list of images thermal images
  */
  static async metaDataImage(img) {
    let attributes = {};
    let latitude;
    let longitude;
    try {
      let metadata = await sharp(`${img.path}/${img.name}`).metadata();
      let dataexitf = exif(metadata.exif);
      if (dataexitf.hasOwnProperty('GPSInfo')) {
        let GPSPosition = dataexitf.GPSInfo;

        latitude = this.convertDMSToDD(
          GPSPosition.GPSLatitude[0],
          GPSPosition.GPSLatitude[1],
          GPSPosition.GPSLatitude[2],
          GPSPosition.GPSLatitudeRef
        );
        longitude = this.convertDMSToDD(
          GPSPosition.GPSLongitude[0],
          GPSPosition.GPSLongitude[1],
          GPSPosition.GPSLongitude[2],
          GPSPosition.GPSLongitudeRef
        );
        attributes['latitude'] = latitude;
        attributes['longitude'] = longitude;
      }
      if (dataexitf.hasOwnProperty('Photo') && dataexitf.Photo.hasOwnProperty('UserComment')) {
        let mystring = dataexitf.Photo.UserComment.toString('utf8').replace(/\u0000/g, '');
        let userdata = JSON.parse(mystring.slice(7).trim());
        if (userdata.hasOwnProperty('MaxTemp')) {
          attributes['measures'] = [{ name: 'TempMax', value: userdata.MaxTemp }];
        }
      }
      return { state: false, attributes: attributes };
    } catch (e) {
      console.error(e); // should contain code (exit code) and signal (that caused the termination).
      return { state: false, attributes: attributes };
    }
  }

  static getNormalSize({ width, height, orientation }) {
    return (orientation || 0) >= 5 ? { width: height, height: width } : { width, height };
  }

  /*
  / convert from DMS to DD
  */

  static convertDMSToDD(degrees, minutes, seconds, direction) {
    var dd = degrees + minutes / 60 + seconds / (60 * 60);
    if (direction == 'S' || direction == 'W') {
      dd = dd * -1; // Convert to negative if south or west
    }
    return dd;
  }

  static async ProcessThermalImages(img) {
    console.log('process img' + img.ref);
    let processImg = JSON.parse(JSON.stringify(img));
    processImg.name = img.name.slice(0, -4) + '_process.jpg';

    try {
      const { stdout, stderr } = await exec(
        `conda run -n DJIThermal ${processThermalsSrc} -i "${img.path}/${img.name}" -o "${processImg.path}${processImg.name}" `
      );
      console.log('stdout:', stdout);
      console.log('stderr:', stderr);
      return { state: true, image: processImg };
    } catch (e) {
      console.error(e); // should contain code (exit code) and signal (that caused the termination).
      return { state: false, image: {} };
    }
  }

  static async listFiles({ uavId, missionId }) {
    console.log('devices acction ' + uavId);
    return [];
  }
}
